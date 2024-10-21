import pymap3d

class GPSManager:
    def __init__(self, home_geo):
        self.home = home_geo

    def geo_to_ned(self, geo):
        return pymap3d.ned.geodetic2ned(geo[0], geo[1], geo[2], self.home[0], self.home[1], self.home[2])

    def ned_to_geo(self, ned):
        return pymap3d.ned.ned2geodetic(ned[0], ned[1], ned[2], self.home[0], self.home[1], self.home[2])