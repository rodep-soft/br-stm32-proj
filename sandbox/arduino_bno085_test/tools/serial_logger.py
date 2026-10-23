import argparse
import csv
from pathlib import Path

import serial


def main():
	parser = argparse.ArgumentParser(description="Save BNO085 timestamps from serial to CSV.")
	parser.add_argument("port", help="Serial port, for example /dev/ttyACM0")
	parser.add_argument("--baud", type=int, default=115200)
	parser.add_argument("--output", type=Path, default=Path("logs/bno085_timestamps.csv"))
	args = parser.parse_args()

	args.output.parent.mkdir(parents=True, exist_ok=True)
	write_header = not args.output.exists() or args.output.stat().st_size == 0

	print(f"Reading {args.port} at {args.baud} baud; saving to {args.output}")
	try:
		with serial.Serial(args.port, args.baud, timeout=1) as port:
			with args.output.open("a", newline="", encoding="utf-8") as output:
				writer = csv.writer(output)
				if write_header:
					writer.writerow(["timestamp_us"])

				while True:
					line = port.readline().decode("ascii", errors="ignore").strip()
					if not line.isdecimal():
						continue

					writer.writerow([int(line)])
					output.flush()
	except KeyboardInterrupt:
		print("\nStopped.")


if __name__ == "__main__":
	main()
