#!/usr/bin/env python3
"""
USB-to-CAN (CH340 ATプロトコル @ 921600bps) 受信 & DBCデコードロガー

フレーム構造 (17 bytes @ DLC=8):
  [0:2]   0x41 0x54 ("AT")      : ヘッダ (2 bytes)
  [2:6]   IDフィールド          : CAN ID / フラグ (4 bytes)
  [6]     DLC                   : データ長 0..8 (1 byte)
  [7:7+N] Data                  : CANペイロード (DLC bytes)
  [7+N:9+N] 0x0D 0x0A ("\r\n")  : フッタ (2 bytes)
"""

import argparse
import csv
from datetime import datetime
import glob
from pathlib import Path
import sys
import time

import cantools
import serial


def auto_detect_port() -> str:
    """接続されているシリアルポート (/dev/ttyUSB* 等) を自動検出する"""
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if candidates:
        return candidates[0]
    return "/dev/ttyUSB0"


def decode_can_id(id_bytes: bytes, known_std_ids: set[int]) -> tuple[int, bool, bool]:
    """
    ATプロトコルの4バイトIDフィールドから (arbitration_id, is_extended, is_remote) を復元する。

    対応フォーマット:
    1. Little-Endian (StdId << 18) 形式 (例: ID 0x100 -> 00 00 00 04)
    2. STM32 RIR/TIR Big-Endian 形式:
       - 標準フレーム (bit2==0): StdId = raw_be >> 21
       - 拡張フレーム (bit2==1): ExtId = (raw_be >> 3) & 0x1FFFFFFF
    """
    raw_le = int.from_bytes(id_bytes, byteorder="little")
    raw_be = int.from_bytes(id_bytes, byteorder="big")

    # 1) Little-Endian (StdId << 18) でDBC既知IDに一致する場合 (00 00 00 04 -> 0x100)
    le_std_id = (raw_le >> 18) & 0x7FF
    if le_std_id in known_std_ids and (raw_le & 0x3FFFF) == 0:
        return le_std_id, False, False

    # 2) 標準的な AT プロトコル (Big-Endian RIR 形式)
    is_extended = bool(raw_be & 0x04)
    is_remote = bool(raw_be & 0x02)
    if not is_extended:
        std_id = (raw_be >> 21) & 0x7FF
        if std_id in known_std_ids or std_id != 0:
            return std_id, False, is_remote
    else:
        ext_id = (raw_be >> 3) & 0x1FFFFFFF
        if ext_id != 0:
            return ext_id, True, is_remote

    # 3) アダプタが拡張モード設定等で StdId を 00 00 00 04 として出力している場合のフォールバック
    if le_std_id != 0:
        return le_std_id, False, False
    if len(known_std_ids) == 1:
        return next(iter(known_std_ids)), False, False

    return 0, is_extended, is_remote


class AtCanStreamParser:
    """シリアルストリームから 'AT' ... '\\r\\n' CANフレームを抽出するパーサ"""

    def __init__(self, known_std_ids: set[int]) -> None:
        self.buf = bytearray()
        self.known_std_ids = known_std_ids

    def feed(self, chunk: bytes) -> list[tuple[int, bool, bool, int, bytes, bytes]]:
        self.buf.extend(chunk)
        frames: list[tuple[int, bool, bool, int, bytes, bytes]] = []

        i = 0
        n = len(self.buf)
        while i <= n - 7:
            # ヘッダ 'AT' (0x41, 0x54) を探す
            if self.buf[i] == 0x41 and self.buf[i + 1] == 0x54:
                dlc = self.buf[i + 6]
                if dlc <= 8:
                    frame_len = 2 + 4 + 1 + dlc + 2
                    if i + frame_len > n:
                        # 次のチャンク到着を待つ
                        break
                    if self.buf[i + frame_len - 2] == 0x0D and self.buf[i + frame_len - 1] == 0x0A:
                        id_bytes = bytes(self.buf[i + 2 : i + 6])
                        data = bytes(self.buf[i + 7 : i + 7 + dlc])
                        arb_id, is_ext, is_rtr = decode_can_id(id_bytes, self.known_std_ids)
                        frames.append((arb_id, is_ext, is_rtr, dlc, data, id_bytes))
                        i += frame_len
                        continue
            i += 1

        if i > 0:
            del self.buf[:i]
        # バッファ肥大化防止
        if len(self.buf) > 4096:
            del self.buf[:-64]

        return frames


def format_signals_with_units(msg_def: cantools.database.Message, decoded: dict) -> str:
    """デコードされた物理値辞書を単位付きの読みやすい文字列に整形する"""
    sig_map = {s.name: s for s in msg_def.signals}
    parts = []
    for name, val in decoded.items():
        sig = sig_map.get(name)
        unit = f" {sig.unit}" if (sig and sig.unit) else ""
        if isinstance(val, float):
            parts.append(f"{name}={val:+8.4f}{unit}")
        else:
            parts.append(f"{name}={val:3d}{unit}")
    return " | ".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="USB-to-CAN (CH340 ATプロトコル) からCANデータを受信し、DBCでデコードしてログ表示します。"
    )
    parser.add_argument(
        "-p",
        "--port",
        default=None,
        help="シリアルポート (デフォルト: 自動検出 e.g. /dev/ttyUSB0, Windowsなら COM3 等)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=921600,
        help="UARTボーレート (デフォルト: 921600)",
    )
    parser.add_argument(
        "-d",
        "--dbc",
        default=str(Path(__file__).resolve().parent / "bno085_can.dbc"),
        help="DBCファイルのパス (デフォルト: bno085_can.dbc)",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="デコード結果を保存するCSVファイルパス (省略時はコンソール表示のみ)",
    )
    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=0,
        help="受信するフレーム数の上限 (0=無制限, Ctrl+Cで停止)",
    )
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    dbc_path = Path(args.dbc)
    if not dbc_path.exists():
        print(f"[ERROR] DBCファイルが見つかりません: {dbc_path}", file=sys.stderr)
        sys.exit(1)

    db = cantools.database.load_file(str(dbc_path))
    msg_by_id = {m.frame_id: m for m in db.messages}
    known_std_ids = {m.frame_id for m in db.messages if not m.is_extended_frame}

    print("=" * 88)
    print(f" USB-CAN DBC Logger")
    print(f"  Port     : {port} @ {args.baud} bps")
    print(f"  DBC File : {dbc_path.name} ({len(db.messages)} message(s): "
          f"{', '.join(f'{m.name}(0x{m.frame_id:03X})' for m in db.messages)})")
    if args.csv:
        print(f"  CSV Out  : {args.csv}")
    print("=" * 88)

    try:
        ser = serial.Serial(port, baudrate=args.baud, timeout=0.02)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"\n[ERROR] シリアルポート {port} を開けませんでした: {e}", file=sys.stderr)
        print("  -> WSL2の場合、Windows側PowerShell(管理者)で以下を実行してアタッチしてください:", file=sys.stderr)
        print("     usbipd list", file=sys.stderr)
        print("     usbipd bind --busid <BUSID>", file=sys.stderr)
        print("     usbipd attach --wsl --busid <BUSID>", file=sys.stderr)
        sys.exit(1)

    csv_file = None
    csv_writer = None
    csv_headers_written = False
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")

    stream_parser = AtCanStreamParser(known_std_ids=known_std_ids)
    total_frames = 0
    t_start = time.monotonic()

    try:
        while True:
            waiting = ser.in_waiting
            chunk = ser.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue

            frames = stream_parser.feed(chunk)
            for arb_id, is_ext, is_rtr, dlc, data, id_bytes in frames:
                now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                total_frames += 1
                elapsed = max(time.monotonic() - t_start, 1e-6)
                hz = total_frames / elapsed

                msg_def = msg_by_id.get(arb_id)
                if msg_def is not None and len(data) == msg_def.length:
                    decoded = msg_def.decode(data)
                    sig_str = format_signals_with_units(msg_def, decoded)
                    print(
                        f"[{now_str}] ID:0x{arb_id:03X} ({msg_def.name}) | "
                        f"Raw:{data.hex(' ')} | {sig_str} | {hz:5.1f}Hz"
                    )

                    if csv_file is not None:
                        if not csv_headers_written:
                            fieldnames = ["timestamp", "can_id_hex", "message_name", "raw_hex"] + list(decoded.keys())
                            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                            csv_writer.writeheader()
                            csv_headers_written = True
                        row = {
                            "timestamp": now_str,
                            "can_id_hex": f"0x{arb_id:03X}",
                            "message_name": msg_def.name,
                            "raw_hex": data.hex(),
                            **decoded,
                        }
                        csv_writer.writerow(row)
                        csv_file.flush()
                else:
                    print(
                        f"[{now_str}] ID:0x{arb_id:03X} ( raw_id:{id_bytes.hex()} ) | "
                        f"DLC:{dlc} | Raw:{data.hex(' ')} (DBC未定義)"
                    )

                if args.count > 0 and total_frames >= args.count:
                    print(f"\n[INFO] 指定フレーム数 ({args.count}) に到達したため終了します。")
                    return

    except KeyboardInterrupt:
        print(f"\n[INFO] 終了しました (合計受信: {total_frames} frames)。")
    finally:
        ser.close()
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()
