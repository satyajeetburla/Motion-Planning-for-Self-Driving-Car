#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: October 29, 2018
import numpy as np


class VelocityPlanner:
    def __init__(self, time_gap, a_max, slow_speed, stop_line_buffer):
        self._time_gap = time_gap
        self._a_max = a_max
        self._slow_speed = slow_speed
        self._stop_line_buffer = stop_line_buffer
        self._prev_trajectory = [[0.0, 0.0, 0.0]]

    def get_open_loop_speed(self, timestep):
        if timestep < 1e-4:
            return self._prev_trajectory[0][2]

        total_time = 0
        for i in range(len(self._prev_trajectory) - 1):
            dist = np.linalg.norm(np.array(self._prev_trajectory[i + 1][:2]) - np.array(self._prev_trajectory[i][:2]))
            total_time += dist / self._prev_trajectory[i][2]
            if total_time > timestep:
                return self._prev_trajectory[i][2]
        return self._prev_trajectory[-1][2]

    def compute_velocity_profile(self, path, desired_speed, ego_state,
                                 closed_loop_speed, decelerate_to_stop,
                                 lead_car_state, follow_lead_vehicle):

        if decelerate_to_stop:
            return self.decelerate_profile(path, ego_state[3])

        elif follow_lead_vehicle:
            dist_to_lead_car = np.linalg.norm(np.array(lead_car_state[:2]) - np.array([path[0][0], path[1][0]]))
            if dist_to_lead_car < self._time_gap * ego_state[3]:
                return self.follow_profile(path, ego_state[3], desired_speed, lead_car_state)

        return self.nominal_profile(path, ego_state[3], desired_speed)

    def decelerate_profile(self, path, start_speed):
        profile = []
        path_length = sum(
            [np.linalg.norm(np.array(path[i + 1][:2]) - np.array(path[i][:2])) for i in range(len(path) - 1)])
        dist_to_stop = path_length - self._stop_line_buffer
        time_to_stop = start_speed / self._a_max

        for i in range(len(path)):
            profile_speed = start_speed - (self._a_max * time_to_stop * (i / len(path)))
            profile_speed = max(profile_speed, 0.0)
            profile.append([path[i][0], path[i][1], profile_speed])

        return profile

    def follow_profile(self, path, start_speed, desired_speed, lead_car_state):
        profile = []
        time_to_reach_lead_car = (lead_car_state[2] - start_speed) / self._a_max

        for i in range(len(path)):
            if i < len(path) * time_to_reach_lead_car:
                speed = start_speed + self._a_max * (i / len(path))
            else:
                speed = lead_car_state[2]
            profile.append([path[i][0], path[i][1], speed])

        return profile



    def nominal_profile(self, path, start_speed, desired_speed):
        """Computes the velocity profile for the local planner path in a normal
        speed tracking case."""
        profile = []
        total_distance = sum(np.linalg.norm(np.array(path[:2]).T[i] - np.array(path[:2]).T[i - 1], axis=0) for i in
                             range(1, len(path[0])))
        distance_to_accelerate = min(total_distance, calc_distance(start_speed, desired_speed, self._a_max))
        current_distance = 0
        current_speed = start_speed

        for i in range(len(path[0]) - 1):
            delta = np.array([path[0][i + 1] - path[0][i], path[1][i + 1] - path[1][i]])
            delta_distance = np.linalg.norm(delta)
            current_distance += delta_distance

            if current_distance < distance_to_accelerate:
                current_speed = calc_final_speed(current_speed, self._a_max, delta_distance)
            else:
                current_speed = desired_speed

            profile.append([path[0][i], path[1][i], current_speed])

        profile.append([path[0][-1], path[1][-1], desired_speed])

        return profile

    def calc_distance(v_i, v_f, a):
        """Computes the distance given an initial and final speed, with a constant
        acceleration."""
        return (v_f ** 2 - v_i ** 2) / (2 * a)

    def calc_final_speed(v_i, a, d):
        """Computes the final speed given an initial speed, distance travelled,
        and a constant acceleration."""
        discriminant = v_i ** 2 + 2 * a * d
        if discriminant < 0:
            return 0
        return np.sqrt(discriminant)

