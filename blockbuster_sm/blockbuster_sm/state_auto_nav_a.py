from blockbuster_sm.base_state import BaseState
import math
import numpy as np

"""
This class is used to enable the autonomous navigation of the robot through a series of waypoints efficiently.
"""
class AutoNavA(BaseState):
    def __init__(self, name: str, shared_data, action_interface, logger, filename: str):
        super().__init__(name, shared_data, action_interface, logger)
        self.command_file = shared_data.data_path + filename
        self.waypoints = []
        self.distance_threshold_wp = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True
        self.manually_navigating = False
        self.goal_approach_status = None
        self.target_theta_speed = 0.40
        self.target_x_speed = 0.15
        self.start_pose = None

    def enter(self):
        self.logger.info(f"Entering state: {self.name}")
        self.status = "RUNNING"
        commands = self.load_data(self.command_file, "waypoints_a")
        self.waypoints = [command[:3] for command in commands]
        self.distance_threshold_wp = [command[3] for command in commands]

        if self.waypoints:
            self.set_current_waypoint()
            self.perform_auto_navigation()
        else:
            self.logger.error("No valid waypoints found in file: {}".format(self.command_file))
            self.status = "COMPLETED"
            self.reset_navigation_state()

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if not self.goal_reached:
            self.update_navigation_status()

    def set_current_waypoint(self):
        self.current_waypoint = self.waypoints[self.waypoint_index]
        self.goal_reached = False

    def perform_auto_navigation(self):
        if self.current_waypoint:
            self.action_interface('navigate_to_pose', 
                                  goal_x=self.current_waypoint[0], 
                                  goal_y=self.current_waypoint[1], 
                                  goal_theta=self.current_waypoint[2])

    def reset_navigation_state(self):
        self.waypoints = []
        self.distance_threshold_wp = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True
        self.manually_navigating = False
        self.goal_approach_status = None
        self.start_pose = None
        self.alpha_rotation = None

    def update_navigation_status(self):
        dist_current_target = self.distance_to_current_waypoint()
        
        if self.is_reached(dist_current_target):
            check_x, check_y, _ = self.waypoints[self.waypoint_index+1]
            if not self.shared_data.is_circle_free(check_x, check_y, 5, 50):
                self.waypoints.pop(self.waypoint_index+1)
            else:
                self.advance_waypoint()
        elif self.is_last_waypoint(dist_current_target) and not self.manually_navigating:
            self.start_manual_navigation()
        
        if self.manually_navigating:
            self.perform_manual_navigation()
        else:
            self.perform_auto_navigation()        
    
    def distance_to_current_waypoint(self):
        return math.sqrt((self.shared_data.x - self.current_waypoint[0]) ** 2 + 
                         (self.shared_data.y - self.current_waypoint[1]) ** 2)

    def is_reached(self, distance):
        return distance < self.distance_threshold_wp[self.waypoint_index] and self.waypoint_index < len(self.waypoints) - 1

    def is_last_waypoint(self, distance):
        return distance < self.distance_threshold_wp[self.waypoint_index] and self.waypoint_index == len(self.waypoints) - 1

    def advance_waypoint(self):
        self.goal_reached = True
        self.waypoint_index += 1
        self.set_current_waypoint()

    def start_manual_navigation(self):
        self.action_interface('abort_navigation')
        self.manually_navigating = True

    def perform_manual_navigation(self):
        if self.goal_approach_status is None:
            self.start_manual_rotation()
        elif self.goal_approach_status == "MAN_ROT_1":
            self.execute_rotation(self.alpha_rotation, self.target_theta_speed)
            self.transition_to_translation()
        elif self.goal_approach_status == "MAN_TRANS":
            self.execute_translation(self.distance_threshold_wp[self.waypoint_index], self.target_x_speed)
            self.transition_to_final_rotation()
        elif self.goal_approach_status == "MAN_ROT_2":
            self.execute_rotation(self.current_waypoint[2], self.target_theta_speed)
            self.complete_manual_navigation()

    def start_manual_rotation(self):
        self.goal_approach_status = "MAN_ROT_1"
        self.goal_reached = False
        self.alpha_rotation = math.atan2(
            (self.current_waypoint[1] - self.shared_data.y),
            (self.current_waypoint[0] - self.shared_data.x)
        )
        self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
        if self.start_pose[2] > np.pi:
            self.start_pose[2] -= 2 * np.pi

    def transition_to_translation(self):
        if self.goal_reached:
            self.goal_approach_status = "MAN_TRANS"
            self.goal_reached = False

    def transition_to_final_rotation(self):
        if self.goal_reached:
            self.goal_approach_status = "MAN_ROT_2"
            self.goal_reached = False

    def complete_manual_navigation(self):
        if self.goal_reached:
            self.reset_manual_navigation_state()
            self.status = "COMPLETED"

    def reset_manual_navigation_state(self):
        self.start_pose = None
        self.alpha_rotation = None
        self.goal_approach_status = None
        self.manually_navigating = False