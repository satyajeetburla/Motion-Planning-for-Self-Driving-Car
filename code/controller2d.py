#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

import cutils
import numpy as np

class Controller2D:
    def __init__(self, waypoints):
        self.vars = cutils.CUtils()
        self.initialize_controller(waypoints)

    def initialize_controller(self, waypoints):
        self._lookahead_distance = 2.0
        self._current_data = {
            'x': 0,
            'y': 0,
            'yaw': 0,
            'speed': 0,
            'timestamp': 0,
            'frame': 0
        }
        self._desired_speed = 0
        self._commands = {
            'throttle': 0,
            'steer': 0,
            'brake': 0
        }
        self._waypoints = waypoints
        self._steering_conversion = 180.0 / 70.0 / np.pi
        self._start_loop = False

    def update_position(self, x, y, yaw, speed, timestamp, frame):
        self._current_data.update({
            'x': x,
            'y': y,
            'yaw': yaw,
            'speed': speed,
            'timestamp': timestamp,
            'frame': frame
        })
        if frame:
            self._start_loop = True

    def nearest_waypoint(self):
        distances = [np.linalg.norm(np.array([wp[0] - self._current_data['x'], wp[1] - self._current_data['y']])) for wp in self._waypoints]
        return np.argmin(distances)

    def update_target_speed(self):
        idx = self.nearest_waypoint()
        self._desired_speed = self._waypoints[idx][2]

    def lookahead_idx(self, lookahead_distance):
        dist_covered = 0
        start_idx = self.nearest_waypoint()
        for idx in range(start_idx, len(self._waypoints)):
            if dist_covered >= lookahead_distance:
                return idx
            dist_covered += np.linalg.norm(np.array([self._waypoints[idx][0] - self._waypoints[idx-1][0], self._waypoints[idx][1] - self._waypoints[idx-1][1]]))
        return len(self._waypoints) - 1

    def replace_waypoints(self, updated_waypoints):
        self._waypoints = updated_waypoints

    def control_commands(self):
        return self._commands['throttle'], self._commands['steer'], self._commands['brake']

    def adjust_throttle(self, throttle_val):
        self._commands['throttle'] = np.clip(throttle_val, 0, 1)

    def adjust_steer(self, steer_rad):
        steer_value = self._steering_conversion * steer_rad
        self._commands['steer'] = np.clip(steer_value, -1, 1)

    def adjust_brake(self, brake_val):
        self._commands['brake'] = np.clip(brake_val, 0, 1)

    def compute_controls(self):
        if not self._start_loop:
            return
        self.update_target_speed()

    def update_controls(self):

        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        self.vars.create_var('kp', 0.50)
        self.vars.create_var('ki', 0.30)
        self.vars.create_var('integrator_min', 0.0)
        self.vars.create_var('integrator_max', 10.0)
        self.vars.create_var('kd', 0.13)
        self.vars.create_var('kp_heading', 8.00)
        self.vars.create_var('k_speed_crosstrack', 0.00)
        self.vars.create_var('cross_track_deadband', 0.01)
        self.vars.create_var('x_prev', 0.0)
        self.vars.create_var('y_prev', 0.0)
        self.vars.create_var('yaw_prev', 0.0)
        self.vars.create_var('v_prev', 0.0)
        self.vars.create_var('t_prev', 0.0)
        self.vars.create_var('v_error', 0.0)
        self.vars.create_var('v_error_prev', 0.0)
        self.vars.create_var('v_error_integral', 0.0)
        
        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            self.vars.v_error           = v_desired - v
            self.vars.v_error_integral += self.vars.v_error * \
                                          (t - self.vars.t_prev)
            v_error_rate_of_change      = (self.vars.v_error - self.vars.v_error_prev) /\
                                          (t - self.vars.t_prev)

            # cap the integrator sum to a min/max
            self.vars.v_error_integral = \
                    np.fmax(np.fmin(self.vars.v_error_integral, 
                                    self.vars.integrator_max), 
                            self.vars.integrator_min)

            throttle_output = self.vars.kp * self.vars.v_error +\
                              self.vars.ki * self.vars.v_error_integral +\
                              self.vars.kd * v_error_rate_of_change

            # Find cross track error (assume point with closest distance)
            crosstrack_error = float("inf")
            crosstrack_vector = np.array([float("inf"), float("inf")])

            ce_idx = self.get_lookahead_index(self._lookahead_distance)
            crosstrack_vector = np.array([waypoints[ce_idx][0] - \
                                         x - self._lookahead_distance*np.cos(yaw), 
                                          waypoints[ce_idx][1] - \
                                         y - self._lookahead_distance*np.sin(yaw)])
            crosstrack_error = np.linalg.norm(crosstrack_vector)

            # set deadband to reduce oscillations
            # print(crosstrack_error)
            if crosstrack_error < self.vars.cross_track_deadband:
                crosstrack_error = 0.0

            # Compute the sign of the crosstrack error
            crosstrack_heading = np.arctan2(crosstrack_vector[1], 
                                            crosstrack_vector[0])
            crosstrack_heading_error = crosstrack_heading - yaw
            crosstrack_heading_error = \
                    (crosstrack_heading_error + self._pi) % \
                    self._2pi - self._pi

            crosstrack_sign = np.sign(crosstrack_heading_error)

            if ce_idx < len(waypoints)-1:
                vect_wp0_to_wp1 = np.array(
                        [waypoints[ce_idx+1][0] - waypoints[ce_idx][0],
                         waypoints[ce_idx+1][1] - waypoints[ce_idx][1]])
                trajectory_heading = np.arctan2(vect_wp0_to_wp1[1], 
                                                vect_wp0_to_wp1[0])
            else:
                vect_wp0_to_wp1 = np.array(
                        [waypoints[0][0] - waypoints[-1][0],
                         waypoints[0][1] - waypoints[-1][1]])
                trajectory_heading = np.arctan2(vect_wp0_to_wp1[1], 
                                                vect_wp0_to_wp1[0])

            heading_error = trajectory_heading - yaw
            heading_error = \
                    (heading_error + self._pi) % self._2pi - self._pi

            steer_output = heading_error + \
                    np.arctan(self.vars.kp_heading * \
                              crosstrack_sign * \
                              crosstrack_error / \
                              (v + self.vars.k_speed_crosstrack))


            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.x_prev       = x
        self.vars.y_prev       = y
        self.vars.yaw_prev     = yaw
        self.vars.v_prev       = v
        self.vars.v_error_prev = self.vars.v_error
        self.vars.t_prev       = t
        
