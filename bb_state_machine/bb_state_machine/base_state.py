from abc import ABC, abstractmethod
import csv
import numpy as np

"""
This class is used to store the different sub-states of the robot.
"""
class BaseState(ABC):
    """
    A base class for all robot states in the state machine.
    """
    def __init__(self, name, shared_data, action_interface, logger):
        self.name = name
        self.shared_data = shared_data
        self.action_interface = action_interface
        self.logger = logger
        self.status = "IDLE"
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.07
        pass

    @abstractmethod
    def enter(self):
        """
        Code to execute when entering the state.
        """
        pass

    @abstractmethod
    def execute(self):
        """
        Code to execute while the state is active.
        """
        pass

    @abstractmethod
    def exit(self):
        """
        Code to execute when exiting the state.
        """
        pass

    def normalize_pose(self, pose):
        x, y, theta = pose
        theta = self.normalize_angle(theta)
        return [x, y, theta]

    def angle_difference(self, target, current):
        diff = (target - current) % (2 * np.pi)
        if diff > np.pi:
            diff -= 2 * np.pi
        return diff

    def execute_rotation(self, angle, angular_speed):
        current_theta = self.normalize_angle(self.shared_data.theta)
        target_theta = self.normalize_angle(angle)
        angle_diff = self.angle_difference(target_theta, current_theta)

        target_theta_speed = 0 if abs(angle_diff) < self.angle_tolerance else np.sign(angle_diff) * angular_speed
        self.goal_reached = abs(angle_diff) < self.angle_tolerance

        self.action_interface('publish_cmd_vel', angular_z=target_theta_speed)

    def normalize_angle(self, angle):
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        return angle

    def execute_translation(self, distance, speed):
        current_distance_to_goal = distance - np.sqrt((self.start_pose[0] - self.shared_data.x) ** 2 + (self.start_pose[1] - self.shared_data.y) ** 2)

        target_x_speed = 0 if current_distance_to_goal <= self.distance_tolerance else np.sign(distance) * speed
        self.goal_reached = current_distance_to_goal <= self.distance_tolerance
            
        self.action_interface('publish_cmd_vel', linear_x=target_x_speed)