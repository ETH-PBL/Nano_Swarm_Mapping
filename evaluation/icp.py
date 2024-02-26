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
import numpy as np
from typing import List

from geometry import Vector
from homogeneous import v2t


class PointCloud:
    def __init__(self, points: List[Vector]):
        self.points = points

    def __add__(self, other: "PointCloud"):
        return PointCloud(self.points + other.points)

    def plot(self, ax, color="green", normals=False, normals_color="grey", markersize=None):
        for point in self.points:
            point.plot(ax, color=color, markersize=markersize)
        if normals:
            points = ICP.from_point_cloud(self)
            normals = ICP.compute_normals(points)
            for i in range(len(normals)):
                start = points[:, i]
                end = start + normals[i] * 0.1
                ax.plot([start[0], end[0]], [start[1], end[1]], color=normals_color)


# ICP algorithm based on the notebook by Igor Bogoslavskyi:
# https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
class ICP:

    @staticmethod
    def from_point_cloud(cloud: PointCloud):
        return np.array([[p.x for p in cloud.points], [p.y for p in cloud.points]])

    @staticmethod
    def to_point_cloud(arr) -> PointCloud:
        return PointCloud([Vector(p[0], p[1]) for p in arr.T])

    @staticmethod
    def compute_normals(points):
        normals = []
        for i in range(points.shape[1]):
            cur_point = prev_point = next_point = points[:, i]
            min_dist = math.inf
            snd_min_dist = math.inf
            for j in range(points.shape[1]):
                if i == j: continue
                dist = np.linalg.norm(cur_point - points[:, j])
                if dist < min_dist:
                    prev_point = next_point
                    next_point = points[:, j]
                    snd_min_dist = min_dist
                    min_dist = dist
                elif dist < snd_min_dist:
                    prev_point = points[:, j]
                    snd_min_dist = dist
            normal = np.flip(next_point - prev_point) * np.array([-1, 1])
            normal /= np.linalg.norm(normal)
            normals.append(normal)
        return normals

    @staticmethod
    def compute_correspondences(P, Q):
        """For each point in P find closest one in Q."""
        p_size = P.shape[1]
        q_size = Q.shape[1]
        correspondences = []
        # p_normals = ICP.compute_normals(P)
        # q_normals = ICP.compute_normals(Q)
        for i in range(p_size):
            p_point = P[:, i]
            min_dist = 0.2
            chosen_idx = -1
            for j in range(q_size):
                # Check if the normals are more than 45° to each other
                # if math.fabs(p_normals[i].dot(q_normals[j])) < 0.7071067812: continue
                q_point = Q[:, j]
                dist = np.linalg.norm(q_point - p_point)
                if dist < min_dist:
                    min_dist = dist
                    chosen_idx = j
            if chosen_idx >= 0:
                correspondences.append((i, chosen_idx))
        return correspondences

    @staticmethod
    def center_data(data, indices):
        reduced_data = data[:, indices]
        center = np.array([reduced_data.mean(axis=1)]).T
        return center, data - center

    @staticmethod
    def compute_cross_covariance(P, Q, correspondences, kernel=lambda diff: 1.0):
        cov = np.zeros((2, 2))
        exclude_indices = []
        for i, j in correspondences:
            p_point = P[:, [i]]
            q_point = Q[:, [j]]
            weight = kernel(p_point - q_point)
            if weight < 0.01: exclude_indices.append(i)
            cov += weight * q_point.dot(p_point.T)
        return cov, exclude_indices

    @staticmethod
    def icp_svd(orig: PointCloud, moved: PointCloud, max_iterations=20, kernel=lambda diff: 1.0):

        # Convert point clouds to numpy matrices
        P = ICP.from_point_cloud(moved)
        Q = ICP.from_point_cloud(orig)
        P_copy = P.copy()

        output_clouds = [moved]
        output_correspondences = [[]]
        T = v2t((0, 0, 0))

        num_iterations = 0
        num_converged_iterations = 0

        for i in range(max_iterations):
            # Compute correspondences
            correspondences = ICP.compute_correspondences(P_copy, Q)

            # Filter correspondences according to distribution
            # norm = lambda tpl: np.linalg.norm(P_copy[:, tpl[0]] - Q[:, tpl[1]])
            # correspondences_norms = list(map(norm, correspondences))
            # limit = np.average(correspondences_norms) + np.std(correspondences_norms)
            # correspondences = list(filter(lambda tpl: norm(tpl) < limit, correspondences))

            # Find the center of the point cloud we are matching against (based on correspondences)
            center_of_Q, Q_centered = ICP.center_data(Q, [index for _, index in correspondences])

            # Find the center of the moving cloud (based on correspondences)
            center_of_P, P_centered = ICP.center_data(P_copy, [index for index, _ in correspondences])

            # Computer the cross variance on centered data
            cov, exclude_indices = ICP.compute_cross_covariance(P_centered, Q_centered, correspondences, kernel)

            # Compute SVD and transformation
            U, S, V_T = np.linalg.svd(cov)
            R = U.dot(V_T)
            t = center_of_Q - R.dot(center_of_P)
            P_copy_new = R.dot(P_copy) + t
            delta = np.linalg.norm(P_copy_new - P_copy)
            P_copy = P_copy_new

            R_h = np.zeros((3, 3))
            R_h[0:2, 0:2] = R
            R_h[2, 2] = 1
            T = v2t((t[0, 0], t[1, 0], 0)) @ R_h @ T

            output_clouds.append(ICP.to_point_cloud(P_copy))
            output_correspondences.append(correspondences)

            # Convergence checking
            num_iterations += 1
            if delta < 0.01:
                num_converged_iterations += 1
                if num_converged_iterations >= 3:
                    break

        return output_clouds, output_correspondences, T, num_iterations - num_converged_iterations
