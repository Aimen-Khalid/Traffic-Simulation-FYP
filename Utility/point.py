from shapely import Point
from math import sqrt


class ScaledPoint:
    def __init__(self, x, y):
        self.point = Point(x, y)
        self.x = x
        self.y = y

    def __mul__(self, s):
        return ScaledPoint(self.point.x * s, self.point.y * s)

    def __truediv__(self, scalar):
        return ScaledPoint(self.point.x / scalar, self.point.y / scalar) if scalar != 0 else self

    def __add__(self, other):
        return ScaledPoint(self.point.x + other.point.x, self.point.y + other.point.y)

    def __sub__(self, other):
        return ScaledPoint(self.point.x - other.point.x, self.point.y - other.point.y)

    def __repr__(self):
        return f"({str(self.point.x)},{str(self.point.y)})"

    def norm(self):
        return sqrt(self.point.x**2+self.point.y**2)

    def get_x(self):
        return self.point.x

    def get_y(self):
        return self.point.y



