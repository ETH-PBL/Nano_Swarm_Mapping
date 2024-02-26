#
# Copyright (C) 2023 ETH Zurich
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the GPL-3.0 license.  See the LICENSE file for details.
#
# Author: Carl Friess
#


import csv
import math
import struct

from geometry import Vector
from icp import PointCloud


def load_poses_cf(filename):
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        poses = {}
        for row in reader:
            if row[0] not in poses:
                poses[row[0]] = []
            poses[row[0]].append([float(row[1]), float(row[2]), float(row[3])])
        return poses


def load_scans(filename):
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        scans = {}
        for row in reader:
            scans[row[0]] = bytes.fromhex(row[1])
        return scans


def convert_vicon_pose(input):
    w, x, y, z = input[3], input[4], input[5], input[6]
    yaw = math.asin(2 * x * y + 2 * z * w)
    return [input[0], input[1], yaw]


def load_poses_vicon(filename):
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        poses = {}
        for row in reader:
            poses[row[0]] = convert_vicon_pose(list(map(float, row[1:])))
        return poses


def extract_points(scan, x, y, yaw, max_measurement=math.inf):
    offsets = [0.02, 0.02, 0.025, 0.025]
    step = -math.pi / 4 / 8

    [pose_id] = struct.unpack("<Hxx", scan[:4])

    points = []
    for frame in [scan[4 + i:4 + i + 76] for i in range(0, 15 * 76, 76)]:
        [dx, dy, dyaw] = struct.unpack("<fff", frame[64:])
        frame_x = x + dx
        frame_y = y + dy
        frame_yaw = yaw + dyaw
        for dir in range(4):
            angle = -4 * step + step / 2
            dir_yaw = frame_yaw
            if dir == 1:
                dir_yaw += math.pi
            elif dir == 2:
                dir_yaw += math.pi / 2
            elif dir == 3:
                dir_yaw -= math.pi / 2
            for col in range(8):

                [measurement] = struct.unpack("<h", frame[dir * 8 * 2 + col * 2:dir * 8 * 2 + col * 2 + 2])
                if measurement < 0 or measurement / 1000 > max_measurement: continue

                dist_x = measurement / 1000
                dist_y = math.tan(angle) * dist_x
                dist_x += offsets[dir]
                p_x = frame_x + dist_x * math.cos(dir_yaw) - dist_y * math.sin(dir_yaw)
                p_y = frame_y + dist_x * math.sin(dir_yaw) + dist_y * math.cos(dir_yaw)
                points.append(Vector(p_x, p_y))

                angle += step

    return PointCloud(points)