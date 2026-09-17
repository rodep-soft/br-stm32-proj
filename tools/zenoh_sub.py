#!/usr/bin/env python3
"""
Zenoh Subscriber & Test Tool for STM32 Zenoh-Pico Communication
Supports both UDP and TCP, Router / Peer modes, and automatic ROS 2 CDR string decoding.
"""

import argparse
import json
import struct
import sys
import time

try:
    import zenoh
except ImportError:
    print("==> [Zenoh] 'eclipse-zenoh' is not installed. Installing automatically via pip...")
    import subprocess
    cmd = [sys.executable, "-m", "pip", "install", "--user", "--break-system-packages", "eclipse-zenoh", "zenoh-cli"]
    res = subprocess.run(cmd)
    if res.returncode != 0:
        cmd = [sys.executable, "-m", "pip", "install", "--user", "eclipse-zenoh", "zenoh-cli"]
        subprocess.run(cmd)
    try:
        import zenoh
        print("==> [Zenoh] Successfully installed and loaded 'eclipse-zenoh'!")
    except ImportError:
        print("Error: Failed to automatically install 'eclipse-zenoh'.")
        print("Please run manually: pip install --user eclipse-zenoh zenoh-cli")
        sys.exit(1)


def decode_cdr_string(payload_bytes: bytes):
    """
    Attempts to decode a ROS 2 CDR serialized std_msgs/msg/String.
    Format:
      - 4 bytes CDR header: (e.g. 0x00, 0x01, 0x00, 0x00 for Little Endian)
      - 4 bytes length (uint32 LE, including null terminator)
      - N bytes string characters + '\0'
    """
    if len(payload_bytes) < 8:
        return None

    # Check common CDR encapsulation headers
    # Little Endian: [0x00, 0x01, 0x00, 0x00]
    # Big Endian:    [0x00, 0x00, 0x00, 0x00]
    header = payload_bytes[:4]
    if header == b'\x00\x01\x00\x00':
        endian = '<'
    elif header == b'\x00\x00\x00\x00':
        endian = '>'
    else:
        # Not a standard CDR header, try treating as raw string length
        return None

    str_len = struct.unpack(f"{endian}I", payload_bytes[4:8])[0]
    data_start = 8
    data_end = data_start + str_len

    if len(payload_bytes) >= data_end and str_len > 0:
        raw_str = payload_bytes[data_start:data_end]
        # Strip trailing null character if present
        if raw_str.endswith(b'\x00'):
            raw_str = raw_str[:-1]
        try:
            return raw_str.decode('utf-8')
        except UnicodeDecodeError:
            return None

    return None


def format_payload(payload_bytes: bytes) -> str:
    # 1. Try decoding as ROS 2 CDR String
    cdr_str = decode_cdr_string(payload_bytes)
    if cdr_str is not None:
        return f"[ROS2 String] \"{cdr_str}\""

    # 2. Try decoding as UTF-8 plaintext
    try:
        text = payload_bytes.decode('utf-8')
        if text.isprintable():
            return f"[Plaintext] \"{text}\""
    except UnicodeDecodeError:
        pass

    # 3. Fallback: Hex dump
    hex_dump = payload_bytes.hex(' ')
    if len(hex_dump) > 60:
        hex_dump = hex_dump[:60] + "..."
    return f"[Binary ({len(payload_bytes)} bytes)] {hex_dump}"


def listener_callback(sample: zenoh.Sample):
    timestamp = time.strftime("%H:%M:%S")
    key = str(sample.key_expr)
    payload = sample.payload.to_bytes()
    decoded = format_payload(payload)

    print(f"\033[36m[{timestamp}]\033[0m \033[1;32m{key}\033[0m", flush=True)
    print(f"       => \033[1m{decoded}\033[0m", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Zenoh Subscriber & Inspection tool for STM32F767ZI Zenoh-Pico"
    )
    parser.add_argument(
        "--key", "-k",
        default="**",
        help="Key expression to subscribe to (default: '**')"
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["router", "peer", "client"],
        default="router",
        help="Zenoh session mode (default: 'router' to accept client connections from STM32)"
    )
    parser.add_argument(
        "--listen", "-l",
        nargs="+",
        default=["udp/0.0.0.0:7447", "tcp/0.0.0.0:7447"],
        help="Endpoints to listen on (default: udp/0.0.0.0:7447 tcp/0.0.0.0:7447)"
    )
    parser.add_argument(
        "--connect", "-e",
        nargs="+",
        default=[],
        help="Endpoints to connect to (optional)"
    )

    args = parser.parse_args()

    conf = zenoh.Config()
    conf.insert_json5("mode", json.dumps(args.mode))

    if args.listen:
        conf.insert_json5("listen/endpoints", json.dumps(args.listen))
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps(args.connect))

    print("==========================================================", flush=True)
    print("      Zenoh Test Subscriber (STM32 Zenoh-Pico Test)      ", flush=True)
    print("==========================================================", flush=True)
    print(f"Mode       : {args.mode}", flush=True)
    print(f"Listening  : {', '.join(args.listen)}", flush=True)
    if args.connect:
        print(f"Connecting : {', '.join(args.connect)}", flush=True)
    print(f"Subscribed : {args.key}", flush=True)
    print("==========================================================", flush=True)
    print("Waiting for messages from STM32... (Press Ctrl+C to stop)", flush=True)
    print(flush=True)

    try:
        session = zenoh.open(conf)
    except Exception as e:
        if args.mode == "router" and "Address already in use" in str(e):
            print("\033[33m[Notice] Port 7447 is already in use (another router/zenohd is active).\033[0m")
            print("         Switching to client mode and connecting to localhost:7447...", flush=True)
            fallback_conf = zenoh.Config()
            fallback_conf.insert_json5("mode", json.dumps("client"))
            fallback_conf.insert_json5("connect/endpoints", json.dumps(["tcp/127.0.0.1:7447"]))
            try:
                session = zenoh.open(fallback_conf)
                print("\033[32m[Connected] Successfully connected to existing router!\033[0m\n", flush=True)
            except Exception as e2:
                print(f"\033[31mError connecting to router: {e2}\033[0m")
                sys.exit(1)
        else:
            print(f"\033[31mError opening Zenoh session: {e}\033[0m")
            sys.exit(1)

    sub = session.declare_subscriber(args.key, listener_callback)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping Zenoh subscriber...")
    finally:
        sub.undeclare()
        session.close()
        print("Done.")


if __name__ == "__main__":
    main()
