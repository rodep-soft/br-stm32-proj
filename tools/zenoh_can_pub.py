#!/usr/bin/env python3
"""
Zenoh CAN Frame Publisher Test Tool for STM32 CAN Bridge
Publishes ROS 2 can_msgs/msg/Frame over Zenoh to trigger CAN transmission on STM32F767ZI.
"""

import argparse
import json
import struct
import sys
import time

try:
    import zenoh
except ImportError:
    print("Error: 'eclipse-zenoh' is not installed.")
    print("Please run: pip install --user eclipse-zenoh")
    sys.exit(1)

# ROS 2 can_msgs/msg/Frame type metadata
DDS_TYPE = "can_msgs::msg::dds_::Frame_"
TYPE_HASH = "RIHS01_0cdc7d15657573ee6ff151e5cc1943888301e370d957e2c077a154f4755fd976"


def serialize_can_frame(
    can_id: int,
    data: bytes,
    is_extended: bool = False,
    is_rtr: bool = False,
    is_error: bool = False,
    frame_id: str = "can0",
    sec: int = 0,
    nanosec: int = 0
) -> bytes:
    """
    Serializes a can_msgs/msg/Frame to standard ROS 2 Little-Endian CDR.
    """
    buf = bytearray()

    # 1. CDR encapsulation header (Little Endian)
    buf += b'\x00\x01\x00\x00'

    # 2. Header: stamp (sec, nanosec)
    buf += struct.pack('<iI', sec, nanosec)

    # 3. Header: frame_id (string: 4-byte length including '\0', followed by characters and padding to 4 bytes)
    encoded_fid = frame_id.encode('utf-8') + b'\x00'
    buf += struct.pack('<I', len(encoded_fid))
    buf += encoded_fid
    # Pad to 4-byte alignment
    while len(buf) % 4 != 0:
        buf += b'\x00'

    # 4. uint32 id
    buf += struct.pack('<I', can_id)

    # 5. bool is_rtr, bool is_extended, bool is_error, uint8 dlc
    dlc = min(len(data), 8)
    buf += struct.pack('<???B', is_rtr, is_extended, is_error, dlc)

    # 6. uint8[8] data
    padded_data = data[:8].ljust(8, b'\x00')
    buf += padded_data

    return bytes(buf)


def main():
    parser = argparse.ArgumentParser(
        description="Publish ROS 2 can_msgs/msg/Frame over Zenoh to STM32 CAN Bridge"
    )
    parser.add_argument("--domain", "-d", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    parser.add_argument("--topic", "-t", default="can_msgs/frame", help="Topic name (default: 'can_msgs/frame')")
    parser.add_argument("--id", type=lambda x: int(x, 0), default=0x123, help="CAN ID (hex or dec, default: 0x123)")
    parser.add_argument("--data", "-D", default="01,02,03,04,05,06,07,08", help="Comma-separated hex or dec bytes (e.g. '01,02,03,04' or '0xAA,0xBB')")
    parser.add_argument("--extended", "-e", action="store_true", help="Use 29-bit extended CAN ID")
    parser.add_argument("--rtr", action="store_true", help="Send Remote Transmission Request")
    parser.add_argument("--interval", "-i", type=float, default=1.0, help="Publish interval in seconds (0 for single shot)")
    parser.add_argument("--mode", "-m", choices=["router", "peer", "client"], default="router", help="Zenoh mode (default: router)")
    parser.add_argument("--listen", "-l", nargs="+", default=["udp/0.0.0.0:7447", "tcp/0.0.0.0:7447"], help="Endpoints to listen on")
    parser.add_argument("--connect", "-c", nargs="+", default=[], help="Endpoints to connect to")

    args = parser.parse_args()

    # Parse data bytes
    data_bytes = bytes([int(b.strip(), 0) for b in args.data.split(",") if b.strip()])

    # Build key expression: <domain_id>/<topic>/<dds_type>/<type_hash>
    raw_topic = args.topic.lstrip('/')
    key_expr = f"{args.domain}/{raw_topic}/{DDS_TYPE}/{TYPE_HASH}"

    conf = zenoh.Config()
    conf.insert_json5("mode", json.dumps(args.mode))
    if args.listen:
        conf.insert_json5("listen/endpoints", json.dumps(args.listen))
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps(args.connect))

    try:
        session = zenoh.open(conf)
    except Exception as e:
        print(f"Error opening Zenoh session: {e}")
        sys.exit(1)

    print("==========================================================")
    print("      Zenoh CAN Frame Publisher (STM32 CAN Bridge)       ")
    print("==========================================================")
    print(f"KeyExpr   : {key_expr}")
    print(f"CAN ID    : 0x{args.id:03X} ({'Extended' if args.extended else 'Standard'})")
    print(f"CAN Data  : {[hex(b) for b in data_bytes]}")
    print(f"Interval  : {args.interval}s")
    print("==========================================================")

    pub = session.declare_publisher(key_expr)

    count = 0
    try:
        while True:
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            payload = serialize_can_frame(
                can_id=args.id,
                data=data_bytes,
                is_extended=args.extended,
                is_rtr=args.rtr,
                sec=sec,
                nanosec=nanosec
            )

            pub.put(payload)
            print(f"[{time.strftime('%H:%M:%S')}] Published CAN Frame # {count}: ID=0x{args.id:03X}, DLC={len(data_bytes)}, Payload={len(payload)} bytes")
            count += 1

            if args.interval <= 0:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
        pub.undeclare()
        session.close()
        print("Done.")


if __name__ == "__main__":
    main()
