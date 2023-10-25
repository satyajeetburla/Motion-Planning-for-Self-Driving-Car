#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt


class CollisionChecker:
    def __init__(self, circle_offsets, circle_radii, weight):
        self._circle_offsets = circle_offsets
        self._circle_radii = circle_radii
        self._weight = weight

    def collision_check(self, paths, obstacles):
        collision_statuses = np.zeros(len(paths), dtype=bool)

        for idx, path in enumerate(paths):
            for x, y, theta in zip(*path):
                circle_x = [offset * cos(theta) for offset in self._circle_offsets]
                circle_y = [offset * sin(theta) for offset in self._circle_offsets]

                circles = np.array([x + cx for cx in circle_x], [y + cy for cy in circle_y])
                dists = scipy.spatial.distance.cdist(obstacles, circles)
                if np.any(dists < self._circle_radii):
                    break
            else:
                collision_statuses[idx] = True

        return collision_statuses

    def select_best_path_index(self, paths, collision_check_array, goal_state):
        best_idx = None
        min_score = float('inf')

        for idx, path in enumerate(paths):
            if not collision_check_array[idx]:
                continue

            end_x, end_y = path[0][-1], path[1][-1]
            score = sqrt((goal_state[0] - end_x) ** 2 + (goal_state[1] - end_y) ** 2)

            for j, other_path in enumerate(paths):
                if j != idx and not collision_check_array[j]:
                    score += self._weight * np.mean(path[2])

            if score < min_score:
                min_score = score
                best_idx = idx

        return best_idx
