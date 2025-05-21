import math


def geo_offset(lat1, lon1, lat2, lon2):
    """
    Compute the forward (north-south) and right (east-west) offset in meters
    from point (lat1, lon1) to point (lat2, lon2).

    :param lat1: Latitude of the starting point (degrees)
    :param lon1: Longitude of the starting point (degrees)
    :param lat2: Latitude of the destination point (degrees)
    :param lon2: Longitude of the destination point (degrees)
    :return: (forward, right) distances in meters
    """
    # TODO: Find more permanent fix
    lat1 = round(lat1, 8)
    lat2 = round(lat2, 8)

    lon1 = round(lon1, 8)
    lon2 = round(lon2, 8)

    # Earth radius in meters
    R = 6378137.0

    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Forward offset (difference in latitude)
    dlat = lat2 - lat1
    forward = dlat * R

    # Right offset (difference in longitude, corrected by latitude)
    dlon = lon2 - lon1
    right = dlon * R * math.cos(lat1)

    return forward, right


def geo_compute(lat1, lon1, forward, right):
    """
    Compute the destination point (lat2, lon2) in degrees given the starting point (lat1, lon1) and
    the forward and right offsets in meters.

    :param lat1: Latitude of the starting point (degrees)
    :param lon1: Longitude of the starting point (degrees)
    :param forward: Forward offset in meters
    :param right: Right offset in meters
    :return: (lat2, lon2) destination point in degrees
    """
    # Earth radius in meters (WGS84)
    R = 6378137.0

    # Convert degrees to radians
    lat1_rad, lon1_rad = map(math.radians, [lat1, lon1])

    # Compute the destination point in radians
    lat2_rad = lat1_rad + forward / R
    lon2_rad = lon1_rad + right / (R * math.cos(lat1_rad))

    # Convert radians back to degrees
    lat2, lon2 = map(math.degrees, [lat2_rad, lon2_rad])

    return lat2, lon2
