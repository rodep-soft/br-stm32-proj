#!/usr/bin/env python3
"""Read-only macOS AT USB-CAN + ST-LINK VCP capture (stdlib only).

No adapter commands or CAN frames are transmitted. UART diagnostics and raw CAN
bytes are saved alongside CSV and a JSON summary. Defaults match this bench.
"""
import argparse
import collections
import csv
import fcntl
import glob
import json
import os
from pathlib import Path
import re
import select
import struct
import termios
import time


def serial_open(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B9600
        a[6][termios.VMIN] = a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        # Darwin IOSSIOSPEED = _IOW('T', 2, speed_t); speed_t is unsigned long.
        fcntl.ioctl(fd, 0x80085402, struct.pack('L', baud))
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except BaseException:
        os.close(fd)
        raise


def unique_port(pattern):
    ports = glob.glob(pattern)
    if len(ports) != 1:
        raise ValueError(f'Expected one port matching {pattern}, found {ports}')
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--can-port')
    parser.add_argument('--uart-port')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    prefix = args.output or Path(__file__).resolve().parents[1] / 'build' / 'validation' / f'capture_{time.time_ns()}'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    can_port = args.can_port or unique_port('/dev/cu.wchusbserial*')
    uart_port = args.uart_port or unique_port('/dev/cu.usbmodem*')
    can_fd = serial_open(can_port, 921600)
    uart_fd = None
    raw, uart, buffer = bytearray(), bytearray(), bytearray()
    rows, health = [], []
    ids = collections.Counter()
    malformed = 0
    try:
        uart_fd = serial_open(uart_port, 115200)
        start = time.monotonic()
        deadline = start + args.seconds
        print(f'Capturing {args.seconds:g}s: {can_port} + {uart_port}', flush=True)
        while time.monotonic() < deadline:
            ready, _, _ = select.select([can_fd, uart_fd], [], [], min(0.25, max(0, deadline - time.monotonic())))
            for fd in ready:
                data = os.read(fd, 16384)
                if fd == uart_fd:
                    uart.extend(data)
                    continue
                raw.extend(data)
                buffer.extend(data)
                elapsed = time.monotonic() - start
                while len(buffer) >= 7:
                    pos = buffer.find(b'AT')
                    if pos < 0:
                        buffer[:] = buffer[-1:] if buffer[-1:] == b'A' else b''
                        break
                    if pos:
                        del buffer[:pos]
                    if len(buffer) < 7:
                        break
                    dlc = buffer[6]
                    if dlc > 8:
                        malformed += 1
                        del buffer[0]
                        continue
                    size = 9 + dlc
                    if len(buffer) < size:
                        break
                    if buffer[size-2:size] != b'\r\n':
                        malformed += 1
                        del buffer[0]
                        continue
                    packet = bytes(buffer[:size])
                    del buffer[:size]
                    idraw = packet[2:6].hex()
                    ids[idraw] += 1
                    if idraw not in ('00000004', '20000000') or dlc != 8:
                        continue
                    ax, ay, aseq, gz, gseq = struct.unpack('<hhBhB', packet[7:15])
                    rows.append([elapsed, '0x100', idraw, packet[7:15].hex(), ax/256, ay/256, aseq, gz/512, gseq])
        capture_seconds = time.monotonic() - start
    finally:
        os.close(can_fd)
        if uart_fd is not None:
            os.close(uart_fd)
        prefix.with_suffix('.bin').write_bytes(raw)
        prefix.with_suffix('.uart.log').write_bytes(uart)
        with prefix.with_suffix('.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['host_elapsed_s', 'can_id', 'adapter_id_raw', 'payload_hex',
                             'Accel_X_m_s2', 'Accel_Y_m_s2', 'Accel_Seq', 'Gyro_Z_rad_s', 'Gyro_Seq'])
            writer.writerows(rows)
    log = uart.decode(errors='replace')
    for line in log.splitlines():
        if line.startswith('Health '):
            health.append(dict(re.findall(r'(\w+)=([0-9a-f/]+)', line)))
    gaps = [b[0]-a[0] for a, b in zip(rows, rows[1:])]
    result = dict(duration_s=capture_seconds, received_bytes=len(raw), frames=len(rows),
                  raw_id_counts=dict(ids), malformed_packets=malformed,
                  approx_hz=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows) > 1 else 0,
                  max_host_gap_s=max(gaps, default=0),
                  gyro_seq_steps=dict(collections.Counter((b[8]-a[8]) % 256 for a, b in zip(rows, rows[1:]))),
                  boot_banners=log.count('NUCLEO-C542RC BNO085 SPI-to-CAN'),
                  watchdog_reset_logs=log.count('IWDG=1;'),
                  health=health, csv=str(prefix.with_suffix('.csv')), uart_log=str(prefix.with_suffix('.uart.log')))
    prefix.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    summary = {key: value for key, value in result.items() if key != "health"}
    summary["health_samples"] = len(health)
    summary["last_health"] = health[-1] if health else None
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
