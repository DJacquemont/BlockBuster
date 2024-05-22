from bb_state_machine.tools import BaseState
import time
import math
import csv
import numpy as np

class AutoNavT(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.waypoints = []
        self.target_theta_speed = 0.4
        self.target_x_speed = 0.3
        self.tracking = None
        self.tracking_id = None
        self.state = None
        self.rotation_start_time = None
        self.duplo_approach_status = None
        self.goal_reached = None
        self.target_locked = None
        self.start_pose = None
        self.alpha_rotation = None
        self.rotation_accumulated = 0
        self.rotation_target = None

    def enter(self):
        self.logger.info("Entering state: AUTO_NAV_T")
        self.status = "RUNNING"
        self.load_waypoints()
        self.state = "TRACKING"
        self.goal_reached = False
        self.target_locked = False

    def exit(self):
        self.waypoints = []
        self.tracking = None
        self.tracking_id = None
        self.state = None
        self.rotation_start_time = None
        self.duplo_approach_status = None
        self.goal_reached = None
        self.start_pose = None
        self.target_locked = None
        self.alpha_rotation = None
        self.rotation_accumulated = 0
        self.rotation_target = None   

    def execute(self):        
        # TODO: not always the first duplo if the first duplo is closer be not in a valid zone

        if self.state == "TRACKING":
            self.searching_duplo()
        elif self.state == "ROTATION":
            self.executing_360()

    """
    Load the waypoints from the command file
    """
    def load_waypoints(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.waypoints = []
                for line in reader:
                    if len(line) == 2:
                        try:
                            waypoint = (float(line[0]), float(line[1]))
                            self.waypoints.append(waypoint)
                        except ValueError:
                            self.logger.error(f"Invalid number format in line: {line}")
                    else:
                        self.logger.error(f"Malformed line in command file, expected 2 elements but got {len(line)}: {line}")
        except IOError:
            self.logger.error("Failed to read the command file, looking in: " + self.command_file)

    """
    Search for the closest waypoint or duplo and navigate to it
    """
    def searching_duplo(self):
        
        # Find the closest duplo
        dist_closest_duplo = float('inf')
        i_closest_duplo = None
        for i, duplo in self.shared_data.detection_dict.items():
            duplo_x, duplo_y = duplo
            distance = math.sqrt((self.shared_data.x - duplo_x) ** 2 + (self.shared_data.y - duplo_y) ** 2)
            if distance < dist_closest_duplo:
                dist_closest_duplo = distance
                i_closest_duplo = i

        # Find the closest waypoint
        dist_closest_waypoint = float('inf')
        i_closest_waypoint = None
        for i, waypoint in enumerate(self.waypoints):
            waypoint_x, waypoint_y = waypoint
            distance = math.sqrt((self.shared_data.x - waypoint_x) ** 2 + (self.shared_data.y - waypoint_y) ** 2)
            if distance < dist_closest_waypoint:
                dist_closest_waypoint = distance
                i_closest_waypoint = i

        # If we are tracking a waypoint or a duplo, check the distance to the target
        dist_current_target = None
        if self.tracking:
            if self.tracking == "WP":
                target_x, target_y = self.waypoints[self.tracking_id]
                dist_current_target = math.sqrt((self.shared_data.x - target_x) ** 2 + (self.shared_data.y - target_y) ** 2)
            if self.tracking == "DP":
                target_x, target_y = self.shared_data.detection_dict[self.tracking_id]
                dist_current_target = math.sqrt((self.shared_data.x - target_x) ** 2 + (self.shared_data.y - target_y) ** 2)

        # If we are not tracking anything or the distance to the target is too far, update the target
        if not dist_current_target or dist_current_target > 0.4:

            # Decide what to track
            tracking = None
            if isinstance(i_closest_waypoint, int) and isinstance(i_closest_duplo, int):
                if dist_closest_waypoint < dist_closest_duplo:
                    tracking = "WP"
                else:
                    tracking = "DP"
            elif isinstance(i_closest_waypoint, int) and i_closest_duplo is None:
                tracking = "WP"
            elif i_closest_waypoint is None and isinstance(i_closest_duplo, int):
                tracking = "DP"
            else:
                self.logger.info("No waypoints or duplos to track")
                self.status = "COMPLETED"
                return
            
            if tracking == "WP":
                self.tracking = "WP"
                self.tracking_id = i_closest_waypoint
                target_x, target_y = self.waypoints[self.tracking_id]
                self.action_interface('navigate_to_pose', goal_x=target_x, goal_y=target_y, goal_theta=0)
            else:
                self.tracking = "DP"
                self.tracking_id = i_closest_duplo
                target_x, target_y = self.shared_data.detection_dict[self.tracking_id]
                self.action_interface('navigate_to_pose', goal_x=target_x, goal_y=target_y, goal_theta=0)
        else:
            if dist_closest_waypoint <= 0.25 and not self.target_locked:

                if self.tracking == "WP":
                    assert i_closest_waypoint == self.tracking_id
                    self.logger.info(f"Waypoint reached: {self.waypoints[self.tracking_id]}")
                    self.action_interface('abort_navigation')
                    self.waypoints.pop(self.tracking_id)
                    self.tracking = None
                    self.tracking_id = None
                    self.state = "ROTATION"

                elif self.tracking == "DP":
                    self.logger.info(f"Following Duplo, but waypoint reached: {self.waypoints[i_closest_waypoint]}")
                    self.waypoints.pop(i_closest_waypoint)

            if dist_closest_duplo <= 0.25:

                self.logger.info(f"Status: {self.duplo_approach_status}")

                if self.duplo_approach_status is None:
                    self.action_interface('abort_navigation')
                    self.duplo_approach_status = "MAN_ROT"
                    self.target_locked = True
                    self.goal_reached = False
                    self.alpha_rotation = math.atan2((target_y - self.shared_data.y), (target_x - self.shared_data.x))
                    self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
                    if self.start_pose[2] > np.pi:
                        self.start_pose[2] -= 2 * np.pi

                elif self.duplo_approach_status == "MAN_ROT":

                    self.execute_rotation(self.alpha_rotation, self.target_theta_speed)
                    if self.goal_reached:
                        self.duplo_approach_status = "MAN_TRANS"
                        self.goal_reached = False

                elif self.duplo_approach_status == "MAN_TRANS":
                    self.execute_translation(0.25, self.target_x_speed)
                    if self.goal_reached:
                        self.duplo_approach_status = None
                        self.start_pose = None
                        self.target_locked = False
                        self.alpha_rotation = None
                
                        assert i_closest_duplo == self.tracking_id
                        self.logger.info(f"Duplo reached: {self.shared_data.detection_dict[i_closest_duplo]}")
                        del self.shared_data.detection_dict[i_closest_duplo]
                        self.tracking = None
                        self.tracking_id = None
                        

    """
    Calculate the difference between two angles
    """
    def angle_difference(self, target, current):
        diff = (target - current) % (2 * np.pi)
        if diff > np.pi:
            diff -= 2 * np.pi
        return diff

    """
    Execute a 360-degree rotation, looking for duplos
    """
    def executing_360(self):
        target_increment = np.pi / 3  # 10-degree increments

        if self.rotation_target is None:
            self.rotation_accumulated = 0
            self.rotation_target = (self.shared_data.theta + target_increment) % (2 * np.pi)
            if self.rotation_target > np.pi:
                self.rotation_target -= 2 * np.pi

        if self.goal_reached:
            self.goal_reached = False
            self.rotation_accumulated += target_increment

            self.logger.info(f"Rotation target: {self.rotation_target}")
            self.logger.info(f"Rotation accumulated: {self.rotation_accumulated}")
        
            if self.rotation_accumulated >= 2 * np.pi - 0.1:
                self.action_interface('publish_cmd_vel', angular_z=0)
                self.state = "TRACKING"
                self.rotation_accumulated = 0
                self.rotation_target = None
                return
            else:
                self.rotation_target = (self.shared_data.theta + target_increment) % (2 * np.pi)
                if self.rotation_target > np.pi:
                    self.rotation_target -= 2 * np.pi

        self.execute_rotation(self.rotation_target, self.target_theta_speed)

    """
    Execute a rotation to a specific angle
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
    Execute a translation to a specific distance
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