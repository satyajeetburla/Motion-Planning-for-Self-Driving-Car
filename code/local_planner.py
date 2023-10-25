#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import numpy as np
import path_optimizer
import collision_checker
import velocity_planner
from math import sin, cos, pi, sqrt


class LocalPathPlanner:
    def __init__(self, num_paths, path_spacing, collision_offsets, collision_radii,
                 path_weight, time_interval, max_acceleration, reduced_speed,
                 safety_buffer, initial_best_path):
        self._paths_count = num_paths
        self._spacing = path_spacing
        self._optimizer = path_optimizer.PathOptimizer()
        self._collision_detector = collision_checker.CollisionChecker(collision_offsets,
                                                                      collision_radii,
                                                                      path_weight)
        self._speed_planner = velocity_planner.SpeedPlanner(time_interval, max_acceleration,
                                                            reduced_speed, safety_buffer)
        self.previous_optimal_path = initial_best_path

    def compute_goal_states(self, goal_idx, target_state, waypoints, vehicle_state):
        """Generates multiple goal states based on lateral offsets."""
        if goal_idx != len(waypoints) - 1:
            dx = waypoints[goal_idx + 1][0] - waypoints[goal_idx][0]
            dy = waypoints[goal_idx + 1][1] - waypoints[goal_idx][1]
            goal_heading = np.arctan2(dy, dx)
        else:
            dx = waypoints[goal_idx][0] - waypoints[goal_idx - 1][0]
            dy = waypoints[goal_idx][1] - waypoints[goal_idx - 1][1]
            goal_heading = np.arctan2(dy, dx)

        goal_states = []
        for i in range(self._paths_count):
            offset = (i - self._paths_count // 2) * self._spacing
            x_offset = offset * cos(target_state[2] + pi / 2)
            y_offset = offset * sin(target_state[2] + pi / 2)

            transformed_x = target_state[0] - vehicle_state[0]
            transformed_y = target_state[1] - vehicle_state[1]

            local_x = transformed_x * cos(vehicle_state[2]) + transformed_y * sin(vehicle_state[2])
            local_y = transformed_x * -sin(vehicle_state[2]) + transformed_y * cos(vehicle_state[2])

            goal_yaw = goal_heading - vehicle_state[2]
            if goal_yaw > pi:
                goal_yaw -= 2 * pi
            elif goal_yaw < -pi:
                goal_yaw += 2 * pi

            goal_states.append([local_x + x_offset, local_y + y_offset, goal_yaw, target_state[2]])

        return goal_states

    def generate_paths(self, goal_states):
        """Creates multiple paths based on provided goal states."""
        generated_paths = []
        is_path_valid = []
        for state in goal_states:
            path = self._optimizer.generate_spiral_path(state[0], state[1], state[2])
            if np.linalg.norm([path[0][-1] - state[0], path[1][-1] - state[1], path[2][-1] - state[2]]) < 0.1:
                generated_paths.append(path)
                is_path_valid.append(True)
            else:
                is_path_valid.append(False)
        return generated_paths, is_path_valid

    @staticmethod
    def global_transform(paths, vehicle_state):
        """Translates paths to the global frame."""
        global_paths = []
        for path in paths:
            transformed_x = []
            transformed_y = []
            transformed_yaw = []

            for idx in range(len(path[0])):
                x = vehicle_state[0] + path[0][idx] * cos(vehicle_state[2]) - path[1][idx] * sin(vehicle_state[2])
                y = vehicle_state[1] + path[0][idx] * sin(vehicle_state[2]) + path[1][idx] * cos(vehicle_state[2])
                yaw = path[2][idx] + vehicle_state[2]

                transformed_x.append(x)
                transformed_y.append(y)
                transformed_yaw.append(yaw)

            global_paths.append([transformed_x, transformed_y, transformed_yaw])

        return global_paths
