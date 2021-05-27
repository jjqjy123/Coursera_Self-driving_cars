#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

from numpy.lib.function_base import angle
import cutils
import numpy as np
import math

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")                                         # infinite
        desired_speed = 0
        for i in range(len(self._waypoints)):                                # traverse all waypoints to find the clostest point
            dist = np.linalg.norm(np.array([                                 # distance, sqrt(a1**2 + a2**2)
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
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

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)                       
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('KI_item_previous', 0.0)
        self.vars.create_var('v_error_previous', 0.0)
        self.vars.create_var('target_i_previous', 0.0)                         # to be used in Pure pursuit methode

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            # To be adjustabel parameters          
            KP = 1
            KI = 0.5
            KD = 0.01

            # Predefined parameters to be used
            v_error = v_desired - v
            dt = t - self.vars.t_previous
            
            # Calculate every items
            KP_item = KP*v_error
            KI_item = self.vars.KI_item_previous + KI*v_error*dt
            KD_item = KD*(v_error - self.vars.v_error_previous)/dt
            
            # Store datas to be used in next step
            self.vars.KI_item_previous = KI_item    
            self.vars.v_error_previous = v_error
            
            # PID control
            throttle_output = KP_item + KI_item + KD_item
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            method = 1

            if method == 0:                               # Pure pursuit method              
                # Length between rear and front axles
                L = 3

                # Calculate Ld 
                Kdd = 2                                   # To be adjustabel parameters 
                Ld = Kdd*v
                
                # Calculate target point
                target_i = 0
                for i in range(len(waypoints)):
                    distance = np.linalg.norm(np.array([waypoints[i][0] - x, waypoints[i][1] - y]))
                    if abs(distance-Ld)<=10e-3 or i==len(waypoints)-1:
                        target_i = i
                        self.vars.target_i_previous = target_i
                        break

                # Calculate alpha, a_vector·b_vector = |a||b|cos(alpha) = x1x2 + y1y2
                a_vector = [waypoints[target_i][0] - x, waypoints[target_i][1] - y]
                a_distance = np.linalg.norm(np.array([waypoints[target_i][0] - x, waypoints[target_i][1] - y]))
                b_vector = [np.cos(yaw), np.sin(yaw)]
                b_distance = 1
                dot_product = a_vector[0]*b_vector[0] + a_vector[1]*b_vector[1]
                alpha = np.arccos(dot_product/(a_distance*b_distance))
                
                # Calculate direction of alpha, a_vector × b_vector = x1y2 - x2y1
                cross_product = a_vector[0]*b_vector[1] - a_vector[1]*b_vector[0]
                if cross_product >= 0:
                    direction = -1
                else:
                    direction = 1

                # Calculate target point 
                steer_output = direction*np.arctan(2*L*np.sin(alpha)/Ld)
            
            elif method == 1:                      # Stanley Controller
                k = 1                           # To be adjustabel parameters 
                ks = 0
                # Calculate lambda between direction of car and waypoints
                x_delta = waypoints[1][0] - waypoints[0][0]
                y_delta = waypoints[1][1] - waypoints[0][1]
                a_vector = [x_delta, y_delta]
                a_distance = np.linalg.norm(np.array(a_vector))
                b_vector = [np.cos(yaw), np.sin(yaw)]
                b_distance = 1
                dot_product = a_vector[0]*b_vector[0] + a_vector[1]*b_vector[1]
                heading_error = np.arccos(dot_product/(a_distance*b_distance))   

                # Calculate direction of lambda, a_vector × b_vector = x1y2 - x2y1
                cross_product1 = a_vector[0]*b_vector[1] - a_vector[1]*b_vector[0]
                if cross_product1 >= 0:
                    direction1 = -1
                else:
                    direction1 = 1
                # Calculate heading error
                heading_angle = direction1*heading_error

                # Calculate crosscrack error, use (y-y2)/(y1-y2)=(x-x2)/(x1-x2) and e = (a*xc + b*yc +c)/sqrt(a**2 +b**2)
                c_vector = [waypoints[1][0]-x, waypoints[1][1]-y]
                cross_product2 = b_vector[0]*c_vector[1] - b_vector[1]*c_vector[0]
                if cross_product2 >= 0:
                    direction2 = 1
                else:
                    direction2 = -1
                a = y_delta
                b = -x_delta
                c = -y_delta*waypoints[0][0] + x_delta*waypoints[0][1]
                crosstrack_error = abs((a*x + b*y + c)/math.sqrt(a**2 + b**2))
                crosstrack_angle = direction2*np.arctan(k*crosstrack_error/(v + ks))
                           
                steer_output = crosstrack_angle + heading_angle
                if steer_output>0:
                    steer_output = min(steer_output, 1.22)
                else:
                    steer_output = max(steer_output, -1.22)
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t  # Store time to be used in next step
