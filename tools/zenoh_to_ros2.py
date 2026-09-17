#!/usr/bin/env python3
"""
Zenoh to ROS 2 Topic Relay / Bridge
Subscribes to Zenoh router (where STM32 sends) and publishes directly to ROS 2 topic /chatter.
Works seamlessly with any ROS 2 distribution and any RMW!
"""

import argparse
import sys
import struct
import zenoh

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as RosString
except ImportError:
    print("Error: 'rclpy' or 'std_msgs' not found.")
    print("Please run this script inside your ROS 2 environment (source /opt/ros/.../setup.bash).")
    sys.exit(1)


def decode_cdr_string(payload_bytes: bytes) -> str:
    if len(payload_bytes) < 8:
        return payload_bytes.decode('utf-8', errors='replace')
    header = payload_bytes[:4]
    endian = '<' if header == b'\x00\x01\x00\x00' else '>' if header == b'\x00\x00\x00\x00' else None
    if endian:
        str_len = struct.unpack(f"{endian}I", payload_bytes[4:8])[0]
        data = payload_bytes[8:8 + str_len]
        if data.endswith(b'\x00'):
            data = data[:-1]
        return data.decode('utf-8', errors='replace')
    return payload_bytes.decode('utf-8', errors='replace')


class ZenohRos2Relay(Node):
    def __init__(self, router_endpoint: str):
        super().__init__('zenoh_ros2_relay')
        self.publisher = self.create_publisher(RosString, '/chatter', 10)
        self.get_logger().info(f"Connecting to Zenoh router: {router_endpoint}")

        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", f'["{router_endpoint}"]')
        self.session = zenoh.open(conf)

        self.sub = self.session.declare_subscriber("**", self.on_zenoh_sample)
        self.get_logger().info("Relay started! Relaying Zenoh chatter -> ROS 2 /chatter")

    def on_zenoh_sample(self, sample: zenoh.Sample):
        key = str(sample.key_expr)
        if "chatter" in key:
            text = decode_cdr_string(sample.payload.to_bytes())
            msg = RosString()
            msg.data = text
            self.publisher.publish(msg)
            self.get_logger().info(f"[Relayed to ROS 2 /chatter]: \"{text}\"")


def main():
    parser = argparse.ArgumentParser(description="Relay Zenoh messages from STM32 to ROS 2 /chatter")
    parser.add_argument("-e", "--endpoint", default="udp/192.168.50.30:7447", help="Zenoh router endpoint")
    args = parser.parse_args()

    rclpy.init()
    node = ZenohRos2Relay(args.endpoint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
