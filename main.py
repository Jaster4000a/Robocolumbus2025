from dataclasses import dataclass
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sensor_subscribers.imu_subscriber as imu_sub
import sensor_subscribers.gps_subscriber as gps_sub
from coordinate_calculation import haversine

@dataclass
class Waypoint:
    lat: float
    lon: float
    cone: bool

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publisher to /rover/cmd_vel
        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # Initialize subscribers/handlers
        self.gps = gps_sub.SatNavHandler()
        self.imu = imu_sub.ImuHandler()

        # Wait for initial valid GPS data
        current_gps = self.gps.get_latest()
        while current_gps is None or not current_gps['valid']:
            self.get_logger().info("Waiting for valid GPS data...")
            rclpy.spin_once(self.gps, timeout_sec=1.0)
            current_gps = self.gps.get_latest()

        self.get_logger().info(f"Initial GPS fix acquired: {current_gps}")

        current_imu= self.imu.get_latest()
        while current_imu is None:
            self.get_logger().info("Waiting for valid IMU data...")
            rclpy.spin_once(self.imu, timeout_sec=1.0)
            current_imu = self.imu.get_latest()

        self.get_logger().info(f"Initial IMU data acquired: {current_imu}")


        self.waypoints = [
            Waypoint(-22.986507336943177, -43.202501, True)
            # Waypoint(-22.986688820105524, -43.20240046546425, False),
            # Waypoint(-22.986698820105524, -43.20250046546425, False)

        ]

        self.current_index = 0

        # PD control constants
        self.kp_ang = 1.2
        self.kd_ang = 0.2
        self.prev_error = 0.0

        self.kp_lin = 1.0
        self.max_speed = 1  # m/s

        print(self.waypoints)

        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        current_gps = self.gps.get_latest()
        if current_gps is None or not current_gps['valid']:
            self.get_logger().warning("Waiting for valid GPS data...")
            self.stop_robot()
            return

        imu_data = self.imu.get_latest()
        if imu_data is None:
            self.get_logger().warning("Waiting for valid IMU data...")
            self.stop_robot()
            return


        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached.")
            return

        target = self.waypoints[self.current_index]
        distance = haversine(current_gps['lat'], current_gps['lon'], target.lat, target.lon)

        if distance < 2:
            self.get_logger().info(
                f"Reached waypoint {self.current_index} ({target.lat}, {target.lon})"
            )
            self.current_index += 1
            return

        # Move toward waypoint
        self.PD_TOWARDS_WAYPOINT(current_gps, target, imu_data)

    def PD_TOWARDS_WAYPOINT(self, current_gps, waypoint, imu_data):
        # --- Step 1: Compute flat-earth heading to waypoint ---
        meters_per_deg_lat = 111_320
        meters_per_deg_lon = 111_320 * math.cos(math.radians(current_gps['lat']))

        dx = (waypoint.lon - current_gps['lon']) * meters_per_deg_lon  # East
        dy = (waypoint.lat - current_gps['lat']) * meters_per_deg_lat  # North

        # Desired heading (0 = North, clockwise)
        desired_heading = (math.degrees(math.atan2(dx, dy))) % 360

        # --- Step 2: Compute signed heading error ---
        current_heading = imu_data["heading_deg"]  # 0 = North, clockwise
        error = ((desired_heading - current_heading + 540) % 360) - 180  # [-180,180]

        # --- Step 3: PD control for angular velocity ---
        d_error = error - self.prev_error
        angular_z = math.radians(self.kp_ang * error + self.kd_ang * d_error)  # rad/s
        self.prev_error = error

        # --- Step 4: Decide linear velocity ---
        # Rotate in place if error too large
        heading_threshold_deg = 10.0
        if abs(error) > heading_threshold_deg:
            linear_x = 0.0  # rotate only
        else:
            alignment_factor = max(0, math.cos(math.radians(error)))
            linear_x = min(self.kp_lin * alignment_factor, self.max_speed)

        # --- Step 5: Publish command ---
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)

        # --- Step 6: Logging ---
        distance = haversine(current_gps['lat'], current_gps['lon'], waypoint.lat, waypoint.lon)
        self.get_logger().info(
            f"Heading err={error:.2f}°, dist={distance:.2f}m, "
            f"lin={linear_x:.2f}, ang={math.degrees(angular_z):.2f}°, "
            f"des={desired_heading:.2f}°, cur={current_heading:.2f}°"
        )


    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    waypoint = WaypointNavigator()

    # Use a MultiThreadedExecutor so callbacks for the IMU and GPS handler
    # nodes are processed while the WaypointNavigator runs its timer.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(waypoint)
    # add the handler nodes that WaypointNavigator created
    try:
        executor.add_node(waypoint.gps)
    except Exception:
        pass
    try:
        executor.add_node(waypoint.imu)
    except Exception:
        pass

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # destroy nodes
        try:
            waypoint.destroy_node()
        except Exception:
            pass
        try:
            waypoint.gps.destroy_node()
        except Exception:
            pass
        try:
            waypoint.imu.destroy_node()
        except Exception:
            pass

        rclpy.shutdown()


if __name__ == '__main__':
    main()
