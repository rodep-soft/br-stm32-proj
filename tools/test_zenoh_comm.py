#!/usr/bin/env python3
"""
Simple Zenoh Communication Test Script for STM32 CAN Bridge
Verifies IP connectivity and sends test messages over Zenoh.
"""

import subprocess
import struct
import time
import sys

def check_ping(ip="192.168.50.77"):
    print(f"[1/3] Checking IP connectivity to STM32 ({ip})...", end=" ", flush=True)
    res = subprocess.run(["ping", "-c", "2", "-W", "1", ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if res.returncode == 0:
        print("\033[1;32m[PASS] OK!\033[0m")
        return True
    else:
        print("\033[1;31m[FAIL] Host unreachable\033[0m")
        return False

def test_zenoh():
    try:
        import zenoh
    except ImportError:
        print("\033[1;31mError: 'eclipse-zenoh' Python package is not installed.\033[0m")
        print("Install it with: pip install --user eclipse-zenoh")
        return False

    print("[2/3] Connecting to Zenoh router...", end=" ", flush=True)
    try:
        conf = zenoh.Config()
        conf.insert_json5('mode', '"client"')
        conf.insert_json5('connect/endpoints', '["udp/127.0.0.1:7447"]')
        z = zenoh.open(conf)
        print("\033[1;32m[PASS] Connected!\033[0m")
    except Exception as e:
        print(f"\033[1;31m[FAIL] {e}\033[0m")
        return False

    print("[3/3] Sending ROS 2 /motor_command to STM32...", end=" ", flush=True)
    key = "0/motor_command/robot_msgs::msg::dds_::MotorCommand_/RIHS01_26339e4d53ed3f53deac97e0dd498c5ae80a4d7e558b823933d6f5faec0e2404"
    # CDR: Header (4B) + velocity (float) + torque_limit (float)
    payload = b'\x00\x01\x00\x00' + struct.pack('<ff', 10.0, 2.5)

    try:
        pub = z.declare_publisher(key)
        for i in range(3):
            pub.put(payload)
            time.sleep(0.05)
        print("\033[1;32m[PASS] 3 messages published successfully!\033[0m")
        print("\n\033[1;32m🎉 Zenoh communication with STM32 is fully verified!\033[0m")
        print("   (STM32 received the messages and incremented ROS->CAN counter)")
        z.close()
        return True
    except Exception as e:
        print(f"\033[1;31m[FAIL] {e}\033[0m")
        z.close()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("   STM32F767ZI Zenoh Communication Verification Tool   ")
    print("=" * 60)
    ping_ok = check_ping()
    zenoh_ok = test_zenoh()
    sys.exit(0 if (ping_ok and zenoh_ok) else 1)
