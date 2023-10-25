#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import math

# State machine states
FOLLOW_LANE = 0
DECELERATE_TO_STOP = 1
STAY_STOPPED = 2
STOP_THRESHOLD = 0.02
STOP_COUNTS = 10


class BehaviouralPlanner:
    def __init__(self, lookahead, stopsign_fences, lead_vehicle_lookahead):
        self._lookahead = lookahead
        self._stopsign_fences = stopsign_fences
        self._follow_lead_vehicle_lookahead = lead_vehicle_lookahead
        self._state = FOLLOW_LANE
        self._follow_lead_vehicle = False
        self._goal_state = [0.0, 0.0, 0.0]
        self._goal_index = 0
        self._stop_count = 0

    def set_lookahead(self, lookahead):
        self._lookahead = lookahead

    def transition_state(self, waypoints, ego_state, closed_loop_speed):
        if self._state == FOLLOW_LANE:
            closest_dist, closest_idx = get_closest_index(waypoints, ego_state)
            target_idx = self.get_goal_index(waypoints, ego_state, closest_dist, closest_idx)
            target_idx, stop_line_detected = self.check_for_stop_signs(waypoints, closest_idx, target_idx)

            self._goal_index = target_idx
            self._goal_state = waypoints[self._goal_index]

            if stop_line_detected:
                self._goal_state[2] = 0
                self._state = DECELERATE_TO_STOP

        elif self._state == DECELERATE_TO_STOP:
            if closed_loop_speed <= STOP_THRESHOLD:
                self._state = STAY_STOPPED

        elif self._state == STAY_STOPPED:
            if self._stop_count < STOP_COUNTS:
                self._stop_count += 1
            else:
                closest_dist, closest_idx = get_closest_index(waypoints, ego_state)
                target_idx = self.get_goal_index(waypoints, ego_state, closest_dist, closest_idx)
                _, stop_line_detected = self.check_for_stop_signs(waypoints, closest_idx, target_idx)

                if not stop_line_detected:
                    self._state = FOLLOW_LANE
                    self._stop_count = 0

    def get_goal_index(self, waypoints, ego_state, closest_dist, closest_idx):
        total_dist = closest_dist
        idx = closest_idx

        while idx < len(waypoints) - 1:
            segment_len = np.linalg.norm(np.array(waypoints[idx + 1][:2]) - np.array(waypoints[idx][:2]))
            total_dist += segment_len

            if total_dist >= self._lookahead:
                return idx

            idx += 1

        return idx

    def check_for_stop_signs(self, waypoints, start_idx, end_idx):
        for i in range(start_idx, end_idx):
            for fence in self._stopsign_fences:
                if self._intersect(waypoints[i][:2], waypoints[i + 1][:2], fence[:2], fence[2:]):
                    return i, True
        return end_idx, False

    def check_for_lead_vehicle(self, ego_state, lead_car_pos):
        rel_position = np.array(lead_car_pos) - np.array(ego_state[:2])
        distance = np.linalg.norm(rel_position)

        if not self._follow_lead_vehicle:
            if distance < self._follow_lead_vehicle_lookahead:
                rel_direction = rel_position / distance
                ego_direction = [math.cos(ego_state[2]), math.sin(ego_state[2])]

                if np.dot(rel_direction, ego_direction) > (1 / math.sqrt(2)):
                    self._follow_lead_vehicle = True
        else:
            if distance >= self._follow_lead_vehicle_lookahead + 15:
                self._follow_lead_vehicle = False

    @staticmethod
    def _intersect(p1, p2, q1, q2):
        s1 = np.array(q1) - np.array(p1)
        s2 = np.array(p2) - np.array(p1)
        s3 = np.array(q2) - np.array(p1)

        s_dir = np.cross(s2, s1)
        t_dir = np.cross(s2, s3)

        if (s_dir > 0 and t_dir < 0) or (s_dir < 0 and t_dir > 0):
            return True
        return False


def get_closest_index(waypoints, ego_state):
    distances = [np.linalg.norm(np.array([wp[0], wp[1]]) - np.array(ego_state[:2])) for wp in waypoints]
    closest_idx = np.argmin(distances)
    return distances[closest_idx], closest_idx


def pointOnSegment(p1, p2, p3):
    if (p2[0] <= max(p1[0], p3[0]) and (p2[0] >= min(p1[0], p3[0])) and \
       (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
        return True
    else:
        return False
