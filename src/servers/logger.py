#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ImuLogger(Node):
    def __init__(self):
        super().__init__('logger')

        # --- QoS for high-rate sensor topics ---
        qos_profile_sensor = QoSProfile(depth=10)
        qos_profile_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_sensor.durability = QoSDurabilityPolicy.VOLATILE

        # --- Subscription ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "sick_data",
            self.callback,
            qos_profile_sensor
        )

        # --- File setup ---
        self.filename = 'sick_data_log.txt'
        file_exists = os.path.exists(self.filename)
        self.file = open(self.filename, 'a')  # append mode

        if not file_exists:
            self.file.write("# timestamp_sec, col0, col1, col2, col3\n")

        self.get_logger().info(f'Logging /sick_data to {self.filename} (raw 4 floats)')

    def callback(self, msg: Float32MultiArray):
        timestamp = time.time()

        # verify there are exactly 4 floats
        if len(msg.data) != 4:
            self.get_logger().warn(f"Expected 4 values, got {len(msg.data)}")
            return

        # log timestamp + 4 values
        self.file.write(
            f"{timestamp:.6f}, {msg.data[0]:.6f}, {msg.data[1]:.6f}, "
            f"{msg.data[2]:.6f}, {msg.data[3]:.6f}\n"
        )
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
