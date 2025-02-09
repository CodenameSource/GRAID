class Waypoint:
    def __init__(self, lat, lon, alt, hdg=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.hdg = hdg

    def to_dict(self):
        return {'lat': self.lat, 'lon': self.lon, 'alt': self.alt, 'hdg': self.hdg}
