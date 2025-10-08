import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from threading import Lock


class SatNavHandler(Node):
    _instance = None
    _lock = Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(SatNavHandler, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return
        super().__init__("sat_nav_handler")

        self._data_lock = Lock()
        self._latest_data = {
            "timestamp": None,
            "lat": None,
            "lon": None,
            "alt": None,
            "fix_quality": None,
            "velocity": 0.0,
            "track": None,
            "valid": False
        }

        self._prev_fix = None
        self.create_subscription(
            NavSatFix,
            "/rover/sensors/gps_0/fix",
            self._fix_callback,
            10
        )

        self._initialized = True
        self.get_logger().info("SatNavHandler initialized and waiting for GPS data...")

    def _fix_callback(self, msg: NavSatFix):
        with self._data_lock:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
            fix_quality = msg.status.status
            valid = fix_quality >= 0

            velocity, track = 0.0, None
            if self._prev_fix and valid:
                prev_t, prev_lat, prev_lon = self._prev_fix
                dt = t - prev_t
                if dt > 0:
                    dist = self._haversine(prev_lat, prev_lon, lat, lon)
                    velocity = dist / dt
                    track = self._bearing(prev_lat, prev_lon, lat, lon)

            self._latest_data.update({
                "timestamp": t,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "fix_quality": fix_quality,
                "velocity": velocity,
                "track": track,
                "valid": valid
            })

            self._prev_fix = (t, lat, lon)
        self.get_logger().debug(f"Received GPS fix: ({lat:.6f}, {lon:.6f})")

    def get_latest(self):
        with self._data_lock:
            return dict(self._latest_data)

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1, lon1, lat2, lon2):
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360


# Example of how to use the SatNavHandler
def main(args=None):
    rclpy.init(args=args)
    handler = SatNavHandler()

    # Timer callback every 1 s to check and print latest data
    def print_latest():
        data = handler.get_latest()
        if data["valid"]:
            handler.get_logger().info(
                f"[{data['timestamp']:.2f}] Lat: {data['lat']:.6f}, Lon: {data['lon']:.6f}, "
                f"Alt: {data['alt']:.2f}, Fix: {data['fix_quality']}, "
                f"Speed: {data['velocity']:.2f} m/s, Track: {data['track']}"
            )
        else:
            handler.get_logger().warn("Waiting for valid GPS fix...")

    handler.create_timer(1.0, print_latest)

    try:
        rclpy.spin(handler)  # Keeps node alive and processes callbacks
    except KeyboardInterrupt:
        pass
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
