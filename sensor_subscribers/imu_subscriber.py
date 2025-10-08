import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from threading import Lock, Thread
import time
import math


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw in degrees."""
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


class ImuHandler(Node):
    _instance = None
    _lock = Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(ImuHandler, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return

        super().__init__("imu_handler")

        self.latest_data = None
        self.data_lock = Lock()

        self.subscription = self.create_subscription(
            Imu,
            "/rover/sensors/imu_0/data",
            self.imu_callback,
            10,
        )

        self._initialized = True
        self.get_logger().info("IMU Handler node initialized and subscribed to /rover/sensors/imu_0/data")

    def imu_callback(self, msg: Imu):
        heading_deg = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        with self.data_lock:
            self.latest_data = {
                "timestamp": self.get_clock().now().to_msg(),
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "heading_deg": heading_deg,
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
            }

    def get_latest(self):
        with self.data_lock:
            return self.latest_data.copy() if self.latest_data else None


def start_imu_handler():
    rclpy.init()
    node = ImuHandler()
    return node


def spin_node(node):
    rclpy.spin(node)


if __name__ == "__main__":
    imu_node = start_imu_handler()

    # Run ROS spin in background
    thread = Thread(target=spin_node, args=(imu_node,), daemon=True)
    thread.start()

    print("Waiting for IMU data...\n")

    try:
        while True:
            data = imu_node.get_latest()
            if data:
                print(f"Timestamp: {data['timestamp'].sec}.{data['timestamp'].nanosec}")
                print(f"Orientation: {data['orientation']}")
                print(f"Heading (deg, Z/North): {data['heading_deg']:.2f}")
                print(f"Angular velocity: {data['angular_velocity']}")
                print(f"Linear acceleration: {data['linear_acceleration']}")
                print("-" * 50)
            else:
                print("No IMU data yet...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down IMU handler...")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()
