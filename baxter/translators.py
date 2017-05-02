# coding=utf-8

# Переводить координаты  из координат Baxter'a(rpy)
# в пиксели
class CoordinatesTranslator:
    w = 0
    h = 0
    aim_x = 0
    aim_y = 0
    dmove = 0
    aim = None

    """Translate coordinates from baxter rpy to px"""

    def __init__(self, aim, dm):
        """
        :param aim: start moving position 
        :param size: size of image 
        :param dmove: translator coefficient 
        """
        self.dmove = dm
        self.aim_x, self.aim_y = aim

    def set_resolution(self, resolution):
        self.w, self.h = resolution

    def translate(self, x_px, y_px, z_px):
        x = self.dmove * (y_px - self.h * self.aim_y) / z_px
        y = self.dmove * (x_px - self.w * self.aim_x) / z_px
        return x, y
