from bb_state_machine.tools import BaseState
import csv
import math
import numpy as np

"""
This state is used to navigate the robot to a series of waypoints defined in a csv file.
"""
class AutoNavA(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.waypoints = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True
        self.manually_navigating = False
        self.goal_approach_status = None
        self.target_theta_speed = 0.4
        self.target_x_speed = 0.3
        self.start_pose = None

    def enter(self):
        self.logger.info("Entering state: AUTO_NAV_A")
        self.status = "RUNNING"
        self.load_waypoints()
        self.waypoint_index = 0
        if self.waypoints:
            self.current_waypoint = self.waypoints[self.waypoint_index]
            self.goal_reached = False
            self.action_interface('navigate_to_pose', goal_x=self.current_waypoint[0], goal_y=self.current_waypoint[1], goal_theta=self.current_waypoint[2])

    def exit(self):
        self.waypoints = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True
        self.manually_navigating = False
        self.goal_approach_status = None
        self.start_pose = None
        self.alpha_rotation = None

    def execute(self):
        if not self.goal_reached:

            # Checking if the robot has reached the current waypoint and if it is the last waypoint
            dist_current_target = math.sqrt((self.shared_data.x - self.current_waypoint[0]) ** 2 + (self.shared_data.y - self.current_waypoint[1]) ** 2)
            if dist_current_target < 0.4 and self.waypoint_index < len(self.waypoints)-1:
                self.goal_reached = True
            elif dist_current_target < 0.25 and self.waypoint_index == len(self.waypoints)-1 and not self.manually_navigating:
                self.action_interface('abort_navigation')
                self.manually_navigating = True

            # For the last waypoint, the robot will approach the waypoint manually
            if self.manually_navigating:

                if self.goal_approach_status is None:
                    self.goal_approach_status = "MAN_ROT_1"
                    self.goal_reached = False
                    self.alpha_rotation = math.atan2((self.current_waypoint[1] - self.shared_data.y),(self.current_waypoint[0] - self.shared_data.x))
                    self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
                    if self.start_pose[2] > np.pi:
                        self.start_pose[2] -= 2 * np.pi

                elif self.goal_approach_status == "MAN_ROT_1":

                    self.execute_rotation(self.alpha_rotation, self.target_theta_speed)
                    if self.goal_reached:
                        self.goal_approach_status = "MAN_TRANS"
                        self.goal_reached = False

                elif self.goal_approach_status == "MAN_TRANS":
                    self.execute_translation(0.25, self.target_x_speed)
                    if self.goal_reached:
                        self.goal_approach_status = "MAN_ROT_2"
                        self.goal_reached = False
                
                elif self.goal_approach_status == "MAN_ROT_2":
                        self.execute_rotation(self.current_waypoint[2], self.target_theta_speed)
                        if self.goal_reached:
                            self.start_pose = None
                            self.alpha_rotation = None
                            self.goal_approach_status = None
                            self.manually_navigating = False

            # The waypoint is reached, moving to the next waypoint, or completing the mission
            if self.goal_reached == True:
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    self.current_waypoint = self.waypoints[self.waypoint_index]
                    self.goal_reached = False
                    self.action_interface('navigate_to_pose', goal_x=self.current_waypoint[0], goal_y=self.current_waypoint[1], goal_theta=self.current_waypoint[2])
                else:
                    self.status = "COMPLETED"

    """
    This function reads the waypoints from a csv file and stores them in a list.
    """
    def load_waypoints(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.waypoints = []
                for line in reader:
                    if len(line) == 3:
                        try:
                            command = (float(line[0]), float(line[1]), float(line[2]))
                            self.waypoints.append(command)
                        except ValueError as e:
                            self.logger.error(f"Error converting line to floats: {line} - {e}")
                    else:
                        self.logger.error(f"Malformed line in command file, expected 3 elements but got {len(line)}: {line}")
        except IOError:
            self.logger.error("Failed to read the command file, looking in: " + self.command_file)

    """
    This function calculates the difference between two angles.
    """
    def angle_difference(self, target, current):
        diff = (target - current) % (2 * np.pi)
        if diff > np.pi:
            diff -= 2 * np.pi
        return diff

    """
    This function executes a rotation to a target angle at a given angular speed.
    """
    def execute_rotation(self, angle, angular_speed):
        current_theta = self.shared_data.theta % (2 * np.pi)
        if current_theta > np.pi:
            current_theta -= 2 * np.pi
        target_theta = angle % (2 * np.pi)
        if target_theta > np.pi:
            target_theta -= 2 * np.pi

        angle_diff = self.angle_difference(target_theta, current_theta)

        self.logger.debug(f"Current theta: {current_theta}")
        self.logger.debug(f"Target theta: {target_theta}")
        self.logger.debug(f"Angle difference: {angle_diff}")

        if abs(angle_diff) < 0.07:
            target_theta_speed = 0
            self.goal_reached = True
        else:
            target_theta_speed = np.sign(angle_diff) * angular_speed
        
        self.action_interface('publish_cmd_vel', angular_z=target_theta_speed)

    """
    This function executes a translation to a target distance at a given linear speed.
    """
    def execute_translation(self, distance, speed):
        goal_x = self.start_pose[0] + distance * np.cos(self.start_pose[2])
        goal_y = self.start_pose[1] + distance * np.sin(self.start_pose[2])

        current_distance_to_goal = np.sqrt((goal_x - self.shared_data.x) ** 2 + (goal_y - self.shared_data.y) ** 2)

        self.logger.debug(f"Current pose: {[self.shared_data.x, self.shared_data.y, self.shared_data.theta]}")
        self.logger.debug(f"Current distance to goal: {current_distance_to_goal}")

        distance_tolerance = 0.1
        if current_distance_to_goal <= distance_tolerance:
            self.goal_reached = True
            target_x_speed = 0
        else:
            target_x_speed = np.sign(distance) * speed
            
        self.action_interface('publish_cmd_vel', linear_x=target_x_speed)