from math import radians, sin, cos, sqrt, atan2

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def offset_gps(ref_lat, ref_lon, north_m=0.0, east_m=0.0):
    """
    Offset a reference GPS coordinate by meters north and east.
    Args:
        ref_lat (float): Reference latitude in degrees
        ref_lon (float): Reference longitude in degrees
        north_m (float): Offset in meters toward north (+) or south (-)
        east_m (float): Offset in meters toward east (+) or west (-)
    Returns:
        (new_lat, new_lon): Tuple of new GPS coordinates
    """
    R = 6378137.0  # Earth's radius (WGS84)
    new_lat = ref_lat + (north_m / R) * (180 / 3.141592653589793)
    new_lon = ref_lon + (east_m / (R * cos(radians(ref_lat)))) * (180 / 3.141592653589793)
    return new_lat, new_lon

# Example:

if __name__ == "__main__":
    ref_lat, ref_lon = -22.986687, -43.202501
    new_lat, new_lon = offset_gps(ref_lat, ref_lon, north_m=20, east_m=0)
    print(f"New GPS coordinates: lat={new_lat}, lon={new_lon}")
