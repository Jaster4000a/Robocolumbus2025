import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LaserScanVisualizer(Node):
    def __init__(self):
        super().__init__('laser_scan_visualizer')
        self.subscription = self.create_subscription(
            LaserScan,
            '/rover/sensors/lidar2d_0/scan',
            self.laser_callback,
            10)
        
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.scatter = None

        self.get_logger().info("LaserScanVisualizer node started.")

    def laser_callback(self, msg: LaserScan):
        print(msg.ranges)
        """Convert LaserScan polar coordinates to Cartesian and plot them."""
        # Correctly generate one angle per range reading
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filter out invalid range readings (inf or NaN)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Clear and re-plot
        self.ax.clear()
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.scatter(x, y, s=5, c='blue')
        self.ax.scatter(0, 0, c='red', s=50, label='Lidar Origin')
        self.ax.legend()
        self.ax.set_title("2D LaserScan Visualization")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")

        plt.pause(0.01)  # non-blocking update


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanVisualizer()
    plt.ion()  # interactive mode for live updates
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
