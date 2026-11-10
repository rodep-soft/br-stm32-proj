#!/usr/bin/env python3
"""USB-to-CAN から CAN FD (ID: 0x500, 18B/DLC20B) を受信・デコードして表示するテストスクリプト.

対応バックエンド:
  1. SocketCAN (デフォルト, Linux標準 `socket` のみで動作: CANable / Candlelight / gs_usb / PEAK 等)
     例: python3 tools/read_canfd_0x500.py --iface can0
  2. SLCAN シリアル (USB-CDC / pyserial: CANable slcan ファーム等)
     例: python3 tools/read_canfd_0x500.py --slcan /dev/ttyACM0
  3. WCH AT USB-CAN シリアル
     例: python3 tools/read_canfd_0x500.py --at-serial /dev/ttyUSB0
"""

import argparse
import glob
import os
import socket
import struct
import sys
import time
from typing import List, Optional, Tuple

CAN_ID_IMU = 0x500
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000

# Linux struct canfd_frame: can_id (u32), len (u8), flags (u8), res0 (u8), res1 (u8), data (64B) = 72B
CANFD_FRAME_FMT = "<IBB2x64s"
CANFD_FRAME_SIZE = 72
# Linux struct can_frame: can_id (u32), can_dlc (u8), pad (3B), data (8B) = 16B
CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = 16

# 0x500 payload (18 bytes):
#   0..4  : timestamp_ms (uint32_t)
#   4..5  : seq          (uint8_t)
#   5..6  : status       (uint8_t)
#   6..12 : accel_x,y,z  (int16_t * 3, 0.001 m/s^2)
#  12..18 : gyro_x,y,z   (int16_t * 3, 0.001 rad/s)
IMU_PAYLOAD_FMT = "<IBBhhhhhh"
IMU_PAYLOAD_SIZE = 18
SCALE = 0.001

SLCAN_DLC_TO_LEN = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
    9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64,
}


def decode_imu_0x500(data: bytes) -> Optional[dict]:
    """18バイト以上のペイロードを 0x500 IMU フォーマットとしてデコードする."""
    if len(data) < IMU_PAYLOAD_SIZE:
        return None
    ts_ms, seq, status, ax, ay, az, gx, gy, gz = struct.unpack(
        IMU_PAYLOAD_FMT, data[:IMU_PAYLOAD_SIZE]
    )
    return {
        "timestamp_ms": ts_ms,
        "seq": seq,
        "status": status,
        "accel_m_s2": (ax * SCALE, ay * SCALE, az * SCALE),
        "gyro_rad_s": (gx * SCALE, gy * SCALE, gz * SCALE),
        "raw_accel": (ax, ay, az),
        "raw_gyro": (gx, gy, gz),
    }


class FramePrinter:
    def __init__(self, show_all: bool = False, every: int = 1) -> None:
        self.show_all = show_all
        self.every = max(1, every)
        self.count_0x500 = 0
        self.last_seq: Optional[int] = None
        self.last_ts_ms: Optional[int] = None
        self.last_rx_time: Optional[float] = None
        self.seq_drops = 0
        self.t0: Optional[float] = None

    def handle_frame(self, can_id: int, data: bytes) -> None:
        if can_id != CAN_ID_IMU:
            if self.show_all:
                print(f"[OTHER] ID=0x{can_id:03X} LEN={len(data):2d} DATA={data.hex()}")
            return

        decoded = decode_imu_0x500(data)
        if decoded is None:
            print(f"[0x500 SHORT] LEN={len(data)} DATA={data.hex()}")
            return

        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        rx_dt_ms = (now - self.last_rx_time) * 1000.0 if self.last_rx_time is not None else 0.0
        self.last_rx_time = now

        self.count_0x500 += 1
        seq = decoded["seq"]
        ts_ms = decoded["timestamp_ms"]
        dt_ms = (ts_ms - self.last_ts_ms) & 0xFFFFFFFF if self.last_ts_ms is not None else 0
        self.last_ts_ms = ts_ms
        if self.last_seq is not None:
            step = (seq - self.last_seq) & 0xFF
            if step != 1:
                self.seq_drops += (step - 1) & 0xFF
        self.last_seq = seq

        if (self.count_0x500 - 1) % self.every != 0:
            return

        elapsed = now - self.t0
        hz = (self.count_0x500 - 1) / elapsed if elapsed > 0.001 and self.count_0x500 > 1 else 0.0
        avg_period_ms = 1000.0 / hz if hz > 0.0 else 0.0
        ax, ay, az = decoded["accel_m_s2"]
        gx, gy, gz = decoded["gyro_rad_s"]
        print(
            f"[0x500] ts={ts_ms:8d}ms (dt={dt_ms:2d}ms) "
            f"seq={seq:3d} st={decoded['status']} | "
            f"acc=[{ax:+7.3f}, {ay:+7.3f}, {az:+7.3f}] m/s^2 | "
            f"gyr=[{gx:+7.3f}, {gy:+7.3f}, {gz:+7.3f}] rad/s | "
            f"period={rx_dt_ms:5.1f}ms (avg={avg_period_ms:5.1f}ms, {hz:5.1f}Hz, drop={self.seq_drops})"
        )


def list_detected_devices() -> Tuple[List[str], List[str]]:
    can_ifaces = sorted(
        os.path.basename(p)
        for p in glob.glob("/sys/class/net/can*") + glob.glob("/sys/class/net/slcan*")
    )
    serial_ports = sorted(
        glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/cu.usb*")
        + glob.glob("/dev/cu.wchusbserial*")
    )
    return can_ifaces, serial_ports


def run_socketcan(iface: str, printer: FramePrinter) -> None:
    """Linux SocketCAN (can0 等) から CAN / CAN FD フレームを受信する."""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    try:
        sock.bind((iface,))
    except OSError as exc:
        sock.close()
        can_ifaces, serial_ports = list_detected_devices()
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        print(f"エラー: SocketCAN インターフェース '{iface}' が見つかりません ({exc}).", file=sys.stderr)
        print(f"  検出された CAN IF   : {can_ifaces or 'なし'}", file=sys.stderr)
        print(f"  検出されたシリアル  : {serial_ports or 'なし'}", file=sys.stderr)
        if by_id:
            for p in by_id:
                print(f"    - {os.path.basename(p)} -> {os.path.realpath(p)}", file=sys.stderr)
        print("\n対処方法:", file=sys.stderr)
        print("  1. WSL2 を使用している場合、Windows 側の管理者 PowerShell で USB-to-CAN を WSL にアタッチしてください:", file=sys.stderr)
        print("       usbipd list", file=sys.stderr)
        print("       usbipd attach --wsl --busid <BUSID>", file=sys.stderr)
        print("  2. SocketCAN 対応アダプタ (CANable/gs_usb 等) の場合、インターフェースを UP してください:", file=sys.stderr)
        print(f"       sudo ip link set {iface} up type can bitrate 1000000 dbitrate 1000000 fd on", file=sys.stderr)
        print("  3. シリアル型 USB-CAN アダプタの場合は --slcan または --at-serial を指定してください:", file=sys.stderr)
        print("       python3 read_canfd_0x500.py --slcan /dev/ttyACM1", file=sys.stderr)
        print("       python3 read_canfd_0x500.py --at-serial /dev/ttyUSB0", file=sys.stderr)
        sys.exit(1)
    print(f"Listening on SocketCAN '{iface}' (CAN FD enabled)... Press Ctrl+C to stop.")
    try:
        while True:
            raw = sock.recv(CANFD_FRAME_SIZE)
            if len(raw) == CANFD_FRAME_SIZE:
                can_id_raw, length, _flags, data = struct.unpack(CANFD_FRAME_FMT, raw)
            elif len(raw) == CAN_FRAME_SIZE:
                can_id_raw, length, data = struct.unpack(CAN_FRAME_FMT, raw)
            else:
                continue
            if can_id_raw & (CAN_ERR_FLAG | CAN_RTR_FLAG):
                continue
            can_id = (
                can_id_raw & CAN_EFF_MASK
                if (can_id_raw & CAN_EFF_FLAG)
                else can_id_raw & CAN_SFF_MASK
            )
            printer.handle_frame(can_id, data[:length])
    finally:
        sock.close()


def parse_slcan_line(line: str) -> Optional[Tuple[int, bytes]]:
    """SLCAN 1行 ('d'/'b'/'t' + 3hex ID + 1hex DLC + data_hex) をパースする."""
    line = line.strip()
    if not line:
        return None
    cmd = line[0]
    try:
        if cmd in ("t", "d", "b") and len(line) >= 5:
            can_id = int(line[1:4], 16)
            dlc_code = int(line[4], 16)
            length = SLCAN_DLC_TO_LEN.get(dlc_code, dlc_code)
            hex_data = line[5 : 5 + length * 2]
            if len(hex_data) < length * 2:
                return None
            return can_id, bytes.fromhex(hex_data)
        if cmd in ("T", "D", "B") and len(line) >= 10:
            can_id = int(line[1:9], 16)
            dlc_code = int(line[9], 16)
            length = SLCAN_DLC_TO_LEN.get(dlc_code, dlc_code)
            hex_data = line[10 : 10 + length * 2]
            if len(hex_data) < length * 2:
                return None
            return can_id, bytes.fromhex(hex_data)
    except ValueError:
        return None
    return None


def check_if_stlink(port: str) -> None:
    real = os.path.realpath(port)
    for p in glob.glob("/dev/serial/by-id/*"):
        if os.path.realpath(p) == real and "STLINK" in os.path.basename(p).upper():
            print(
                f"警告: '{port}' は ST-LINK (STM32 のデバッグUART) です。CAN FD 解析ツールではありません！\n"
                f"  現在 WSL に認識されているシリアルデバイスは {os.path.basename(p)} のみです。\n"
                f"  USB-to-CAN 解析ツールを Windows 側 PowerShell (管理者) から `usbipd attach --wsl --busid <BUSID>` で WSL にアタッチしてください。",
                file=sys.stderr,
            )


def run_slcan(port: str, baud: int, printer: FramePrinter) -> None:
    """SLCAN USB-to-CAN シリアルポートから受信する."""
    import serial

    check_if_stlink(port)
    with serial.Serial(port, baudrate=baud, timeout=0.5) as ser:
        # S8 = 1Mbps nominal, Y1 = 1Mbps data (CANable2/slcan FD), O = Open channel
        ser.write(b"C\rS8\rY1\rO\r")
        ser.flush()
        print(f"Listening on SLCAN '{port}' ({baud} bps)... Press Ctrl+C to stop.")
        buf = bytearray()
        while True:
            chunk = ser.read(1024)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx_r = buf.find(b"\r")
                idx_n = buf.find(b"\n")
                indices = [i for i in (idx_r, idx_n) if i >= 0]
                if not indices:
                    break
                sep = min(indices)
                line = buf[:sep].decode("ascii", errors="ignore")
                del buf[: sep + 1]
                parsed = parse_slcan_line(line)
                if parsed is not None:
                    printer.handle_frame(*parsed)


def run_at_serial(port: str, baud: int, printer: FramePrinter) -> None:
    """WCH AT USB-CAN アダプタ ('AT' + 4B ID + 1B len + data + '\\r\\n') から受信する."""
    import serial

    with serial.Serial(port, baudrate=baud, timeout=0.5) as ser:
        print(f"Listening on AT USB-CAN '{port}' ({baud} bps)... Press Ctrl+C to stop.")
        buffer = bytearray()
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)
            while len(buffer) >= 7:
                pos = buffer.find(b"AT")
                if pos < 0:
                    buffer[:] = buffer[-1:] if buffer[-1:] == b"A" else b""
                    break
                if pos > 0:
                    del buffer[:pos]
                if len(buffer) < 7:
                    break
                dlc = buffer[6]
                if dlc > 64:
                    del buffer[0]
                    continue
                size = 9 + dlc
                if len(buffer) < size:
                    break
                if buffer[size - 2 : size] != b"\r\n":
                    del buffer[0]
                    continue
                packet = bytes(buffer[:size])
                del buffer[:size]
                idraw = packet[2:6].hex()
                can_id = (
                    CAN_ID_IMU
                    if idraw in ("00000500", "00050000", "00000014", "14000000", "a0000000", "000000a0")
                    else int(idraw, 16)
                )
                printer.handle_frame(can_id, packet[7 : 7 + dlc])


def run_selftest() -> None:
    """ダミーの 0x500 フレーム (20バイト DLC, 18バイト実データ) をパックしてデコード表示を確認する."""
    print("Running self-test for CAN ID 0x500 decoder...")
    printer = FramePrinter(show_all=True, every=1)
    for i in range(3):
        payload_18 = struct.pack(
            IMU_PAYLOAD_FMT,
            1000 + i * 5,  # timestamp_ms
            i,             # seq
            1,             # status (OK)
            120,           # accel_x = +0.120 m/s^2
            -50,           # accel_y = -0.050 m/s^2
            9807,          # accel_z = +9.807 m/s^2
            10,            # gyro_x  = +0.010 rad/s
            -20,           # gyro_y  = -0.020 rad/s
            305,           # gyro_z  = +0.305 rad/s
        )
        canfd_20 = payload_18 + b"\x00\x00"
        printer.handle_frame(CAN_ID_IMU, canfd_20)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--iface", default="can0", help="SocketCAN インターフェース名 (デフォルト: can0)")
    group.add_argument("--slcan", metavar="PORT", help="SLCAN シリアルポート (例: /dev/ttyACM0)")
    group.add_argument("--at-serial", metavar="PORT", help="WCH AT USB-CAN シリアルポート (例: /dev/ttyUSB0)")
    group.add_argument("--selftest", action="store_true", help="ダミーフレームでデコード表示の単体テストを実行")
    parser.add_argument("--baud", type=int, default=115200, help="シリアル接続時のボーレート (デフォルト: 115200)")
    parser.add_argument("--every", type=int, default=1, help="Nフレームごとに表示 (200Hz時に間引く場合は --every 10 等)")
    parser.add_argument("--all", action="store_true", help="0x500 以外の CAN ID も表示する")
    args = parser.parse_args()

    if args.selftest:
        run_selftest()
        return

    printer = FramePrinter(show_all=args.all, every=args.every)
    try:
        if args.slcan:
            run_slcan(args.slcan, args.baud, printer)
        elif args.at_serial:
            run_at_serial(args.at_serial, args.baud, printer)
        else:
            run_socketcan(args.iface, printer)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
