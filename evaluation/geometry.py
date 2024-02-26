#
# Copyright (C) 2023 ETH Zurich
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the GPL-3.0 license.  See the LICENSE file for details.
#
# Author: Carl Friess
#


import math
from typing import Optional, List
import matplotlib.pyplot as plt
import numpy as np


class Vector:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def __add__(self, other: "Vector") -> "Vector":
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector") -> "Vector":
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float) -> "Vector":
        return Vector(self.x * other, self.y * other)

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def angle(self):
        return math.atan2(self.x, self.y)

    def rotate(self, angle) -> "Vector":
        cos = math.cos(-angle)
        sin = math.sin(-angle)
        return Vector(self.x * cos - self.y * sin, self.x * sin + self.y * cos)

    def to_array(self):
        return np.array([[self.x], [self.y]])

    def plot(self, ax, color="green", markersize=None):
        ax.plot(self.x, self.y, marker="o", color=color, markersize=markersize)


class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.dx, self.dy = x2 - x1, y2 - y1
        self.det = self.dx ** 2 + self.dy ** 2

    def intersection(self, other: "Line") -> Optional[Vector]:
        # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
        x1, y1, x2, y2 = self.x1, self.y1, self.x2, self.y2
        x3, y3, x4, y4 = other.x1, other.y1, other.x2, other.y2
        try:
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
            u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
        except ZeroDivisionError:
            return None

        if 0 <= t <= 1 and 0 <= u <= 1:
            return Vector(x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    def dist(self, point: Vector):
        a = (self.dy * (point.y - self.y1) + self.dx * (point.x - self.x1)) / self.det
        if 0 <= a <= 1:
            return (Vector(self.x1 + a * self.dx, self.y1 + a * self.dy) - point).length()
        return min((Vector(self.x1, self.y1) - point).length(), (Vector(self.x2, self.y2) - point).length())

    def plot(self, ax, color="magenta"):
        ax.plot([self.x1, self.x2], [self.y1, self.y2], color=color)


class Environment:
    def __init__(self, lines: List[Line]):
        self.lines = lines

    def raycast(self, line: Line) -> Optional[Vector]:
        for wall in self.lines:
            point = wall.intersection(line)
            if point is not None:
                return point

    def plot(self, ax=None, size=(16, 10), color="magenta"):
        if ax is None:
            fig = plt.figure(figsize=size)
            ax = fig.add_subplot()
            ax.axis('equal')
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
        for line in self.lines:
            line.plot(ax, color=color)
        return ax
