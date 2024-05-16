from bb_state_machine.tools import BaseState
import time
import math
import csv

class AutoNavT(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.waypoints = []
        self.target_theta_speed = 0.4
        self.tracking = None
        self.tracking_id = None
        self.state = None
        self.rotation_start_time = None

    def enter(self):
        self.logger.info("Entering state: AUTO_NAV_T")
        self.status = "RUNNING"
        self.load_waypoints()
        self.state = "TRACKING"

    def execute(self):        
        # TODO: not always the first duplo if the first duplo is closer be not in a valid zone

        if self.state == "TRACKING":
            self.searching_duplo()
        elif self.state == "ROTATION":
            self.executing_360()

    def exit(self):
        self.waypoints = []
        self.tracking = None
        self.tracking_id = None
        self.state = None


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
            # We are locked on a target
            if dist_closest_waypoint <= 0.25:
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
            if dist_closest_duplo <= 0.05:
                # consider duplo eaten
                assert i_closest_duplo == self.tracking_id
                self.logger.info(f"Duplo reached: {self.shared_data.detection_dict[i_closest_duplo]}")
                self.action_interface('abort_navigation')
                del self.shared_data.detection_dict[i_closest_duplo]
                self.tracking = None
                self.tracking_id = None
        
    
    def executing_360(self):
        current_time = time.time()
        if self.rotation_start_time is None:
            self.rotation_start_time = current_time

        turning_time = 2*math.pi / self.target_theta_speed

        elapsed_time = current_time - self.rotation_start_time

        if elapsed_time >= turning_time:
            self.action_interface('publish_cmd_vel', angular_z=0)
            self.state = "TRACKING"
            self.rotation_start_time = None
        else:
            self.action_interface('publish_cmd_vel', angular_z=self.target_theta_speed)