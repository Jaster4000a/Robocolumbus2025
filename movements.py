import time
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_subscribers.imu_subscriber import ImuHandler


def _angle_error_deg(target: float, current: float) -> float:
    """Return shortest signed error from current to target in degrees."""
    return (target - current + 540.0) % 360.0 - 180.0


def turn_to(
    degrees: int,
    node: Optional[Node] = None,
    imu_node: Optional[ImuHandler] = None,
    pub_topic: str = '/rover/cmd_vel',
    max_angular_speed_deg: float = 90.0,
    tolerance_deg: float = 2.0,
    timeout: float = 10.0,
) -> bool:
    """
    Rotate the robot by `degrees` (relative) using IMU heading feedback.

    Returns True on success (reached target within tolerance), False on timeout or failure.

    Parameters:
    - degrees: signed degrees to rotate (positive = turn to increasing heading)
    - node: optional rclpy.node.Node to use for publishing; if None a temporary node will be created
    - imu_node: optional ImuHandler instance to read heading from; if None a (singleton) ImuHandler will be created
    - pub_topic: topic to publish Twist messages to
    - max_angular_speed_deg: maximum angular velocity in degrees/sec used when rotating
    - tolerance_deg: acceptable final heading error in degrees
    - timeout: maximum seconds to attempt the rotation
    """

    created_node = False
    created_imu = False
    try:
        # Ensure rclpy is initialized; init() is safe to call multiple times in some setups,
        # but protect with try/except in case it's already initialized by the caller.
        try:
            rclpy.init()
        except Exception:
            # ignore if already initialized
            pass

        if node is None:
            node = Node('movement_turner')
            created_node = True

        pub = node.create_publisher(Twist, pub_topic, 10)

        if imu_node is None:
            # ImuHandler is implemented as a singleton. If an instance already
            # exists we should not mark it as created here (so we won't destroy
            # the shared node on exit). If no instance exists, create and mark it.
            if getattr(ImuHandler, '_instance', None) is None:
                imu_node = ImuHandler()
                created_imu = True
            else:
                imu_node = ImuHandler()
                created_imu = False

        # Wait for first IMU reading
        start_wait = time.time()
        imu_data = None
        while time.time() - start_wait < timeout:
            imu_data = imu_node.get_latest()
            if imu_data is not None:
                break
            # give ROS a chance to process callbacks
            try:
                rclpy.spin_once(imu_node, timeout_sec=0.05)
            except Exception:
                pass
            time.sleep(0.05)

        if imu_data is None:
            return False

        initial_heading = imu_data['heading_deg']
        target_heading = (initial_heading + float(degrees)) % 360.0

        start_time = time.time()

        print(f'Turning from {initial_heading:.1f}° to {target_heading:.1f}°')

        success = False
        while time.time() - start_time < timeout:
            # update imu
            try:
                rclpy.spin_once(imu_node, timeout_sec=0.02)
            except Exception:
                pass

            imu_data = imu_node.get_latest()
            if imu_data is None:
                # wait a bit and continue
                time.sleep(0.02)
                continue

            current_heading = imu_data['heading_deg']
            err = _angle_error_deg(target_heading, current_heading)

            if abs(err) <= tolerance_deg:
                success = True
                break

            # proportional angular speed with cap
            speed_deg = max(10.0, min(max_angular_speed_deg, abs(err) * 0.8))
            speed_rad = math.radians(speed_deg)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = math.copysign(speed_rad, err)
            pub.publish(twist)

            # also service the local node so the publisher has a chance to send
            try:
                if node is not imu_node:
                    rclpy.spin_once(node, timeout_sec=0.01)
            except Exception:
                pass

            time.sleep(0.02)

        # stop rotation
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        return success

    finally:
        # cleanup only nodes we created here
        try:
            if created_imu:
                imu_node.destroy_node()
        except Exception:
            pass
        try:
            if created_node:
                node.destroy_node()
        except Exception:
            pass


def drive_for_time(
    linear: float,
    angular: float,
    time: float,
    node: Optional[Node] = None,
    pub_topic: str = '/rover/cmd_vel',
    rate_hz: float = 20.0,
) -> bool:
    """
    Drive with the given linear (m/s) and angular (rad/s) velocities for `time` seconds.

    Parameters:
    - linear: forward speed in m/s
    - angular: angular speed in rad/s
    - time: duration in seconds to drive
    - node: optional rclpy.node.Node; if None a temporary node will be created
    - pub_topic: topic to publish Twist messages to
    - rate_hz: publish rate (Hz)

    Returns True on normal completion.
    """

    created_node = False
    # avoid shadowing the imported time module (parameter named `time`), access via __import__
    tmod = __import__('time')

    try:
        try:
            rclpy.init()
        except Exception:
            pass

        if node is None:
            node = Node('movement_driver')
            created_node = True

        pub = node.create_publisher(Twist, pub_topic, 10)

        start = tmod.time()
        end = start + float(time)
        sleep_dt = 1.0 / float(rate_hz) if rate_hz > 0 else 0.05

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)

        while tmod.time() < end:
            pub.publish(twist)
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:
                pass
            tmod.sleep(sleep_dt)

        # stop
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        pub.publish(stop)
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass

        return True
    finally:
        try:
            if created_node:
                node.destroy_node()
        except Exception:
            pass


def drive_straight_time(
    linear: float,
    time: float,
    imu_node: Optional[ImuHandler] = None,
    node: Optional[Node] = None,
    pub_topic: str = '/rover/cmd_vel',
    rate_hz: float = 20.0,
    kp_heading: float = 0.8,
    max_angular_speed_deg: float = 30.0,
    imu_timeout: float = 2.0,
) -> bool:
    """
    Drive forward at `linear` m/s for `time` seconds while holding heading using IMU.

    - linear: forward speed (m/s)
    - time: duration in seconds
    - imu_node: optional ImuHandler to use for heading feedback
    - node: optional rclpy.node.Node to use for publisher
    - kp_heading: proportional gain applied to heading error (deg -> deg/s)
    - max_angular_speed_deg: cap for angular correction in deg/s
    - imu_timeout: seconds to wait for initial IMU reading before driving open-loop

    Returns True on normal completion.
    """

    created_node = False
    created_imu = False
    tmod = __import__('time')

    try:
        try:
            rclpy.init()
        except Exception:
            pass

        if node is None:
            node = Node('movement_straight')
            created_node = True

        pub = node.create_publisher(Twist, pub_topic, 10)

        # IMU handling (singleton-safe)
        if imu_node is None:
            if getattr(ImuHandler, '_instance', None) is None:
                imu_node = ImuHandler()
                created_imu = True
            else:
                imu_node = ImuHandler()
                created_imu = False

        # Try to get initial heading
        start_wait = tmod.time()
        imu_data = None
        while tmod.time() - start_wait < imu_timeout:
            imu_data = imu_node.get_latest()
            if imu_data is not None:
                break
            try:
                rclpy.spin_once(imu_node, timeout_sec=0.02)
            except Exception:
                pass
            tmod.sleep(0.02)

        if imu_data is None:
            # No IMU — fallback to open-loop drive
            desired_heading = None
        else:
            desired_heading = imu_data['heading_deg']

        start = tmod.time()
        end = start + float(time)
        sleep_dt = 1.0 / float(rate_hz) if rate_hz > 0 else 0.05

        while tmod.time() < end:
            ang = 0.0
            if desired_heading is not None:
                # update IMU reading
                try:
                    rclpy.spin_once(imu_node, timeout_sec=0.0)
                except Exception:
                    pass
                imu_data = imu_node.get_latest()
                if imu_data is not None:
                    cur = imu_data['heading_deg']
                    err = _angle_error_deg(desired_heading, cur)
                    # correction in deg/s
                    corr_deg = kp_heading * err
                    # cap
                    corr_deg = max(-max_angular_speed_deg, min(max_angular_speed_deg, corr_deg))
                    ang = math.radians(corr_deg)

            twist = Twist()
            twist.linear.x = float(linear)
            twist.angular.z = float(ang)
            pub.publish(twist)

            # service node so publisher can flush
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:
                pass

            tmod.sleep(sleep_dt)

        # stop
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        pub.publish(stop)
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass

        return True
    finally:
        try:
            if created_imu:
                imu_node.destroy_node()
        except Exception:
            pass
        try:
            if created_node:
                node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    rclpy.init()
    imu_node = ImuHandler()
    try:
        for i in range(4):
            ok = turn_to(-90, timeout=15.0, imu_node=imu_node)
            print('Success' if ok else 'Failed')
            ok = drive_straight_time(1.0, 3.0, imu_node=imu_node)
            print('Success' if ok else 'Failed')
    finally:
        try:
            imu_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()