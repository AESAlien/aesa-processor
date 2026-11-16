"""ENU geometry; antenna rotation follows the processor's pose convention."""

import math


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(vector):
    return math.sqrt(dot(vector, vector))


def rotate(vector, angles):
    roll, pitch, yaw = map(math.radians, angles)
    x, y, z = vector
    x, z = math.cos(roll) * x + math.sin(roll) * z, -math.sin(roll) * x + math.cos(roll) * z
    y, z = math.cos(pitch) * y - math.sin(pitch) * z, math.sin(pitch) * y + math.cos(pitch) * z
    return (math.cos(yaw) * x + math.sin(yaw) * y, -math.sin(yaw) * x + math.cos(yaw) * y, z)


def inverse_rotate(vector, angles):
    return tuple(dot(vector, rotate(axis, angles)) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))


def enu_to_ecef(vector, latitude, longitude):
    lat, lon = latitude, longitude
    east = (-math.sin(lon), math.cos(lon), 0)
    north = (-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat))
    up = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
    return tuple(sum(vector[j] * axis[i] for j, axis in enumerate((east, north, up))) for i in range(3))


def ecef_to_enu(vector, latitude, longitude):
    return tuple(dot(vector, enu_to_ecef(axis, latitude, longitude)) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))


def sphere_position(latitude, longitude, radius):
    return enu_to_ecef((0, 0, radius), latitude, longitude)


def direct_sphere(latitude, longitude, bearing, angle):
    lat = math.asin(max(-1, min(1, math.sin(latitude) * math.cos(angle) + math.cos(latitude) * math.sin(angle) * math.cos(bearing))))
    lon = longitude + math.atan2(math.sin(bearing) * math.sin(angle) * math.cos(latitude), math.cos(angle) - math.sin(latitude) * math.sin(lat))
    return lat, lon


def antenna_bearing(azimuth, horizontal, up, angles):
    """Intersect the antenna azimuth plane with a fixed ENU horizontal circle."""
    normal = rotate((math.cos(azimuth), -math.sin(azimuth), 0), angles)
    length = math.hypot(normal[0], normal[1])
    if horizontal <= 0 or length < 1e-12:
        raise ValueError("Initial antenna azimuth is undefined for this geometry")
    ratio = -normal[2] * up / (horizontal * length)
    if abs(ratio) > 1:
        raise ValueError("Antenna azimuth and altitude have no intersection")
    phase = math.atan2(normal[1], normal[0])
    candidates = (math.asin(ratio) - phase, math.pi - math.asin(ratio) - phase)
    for bearing in candidates:
        antenna = inverse_rotate((horizontal * math.sin(bearing), horizontal * math.cos(bearing), up), angles)
        if antenna[0] * math.sin(azimuth) + antenna[1] * math.cos(azimuth) > 0:
            return bearing
    raise ValueError("Initial position is behind the specified antenna azimuth")
