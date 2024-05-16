from bb_state_machine.tools import BaseState
import csv

class AutoNavA(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.waypoints = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True
        self.is_nav_complete_counter = 0

    def enter(self):
        self.logger.info("Entering state: AUTO_NAV_A")
        self.status = "RUNNING"
        self.load_waypoints()
        self.waypoint_index = 0
        if self.waypoints:
            self.current_waypoint = self.waypoints[self.waypoint_index]
            self.goal_reached = False
            self.action_interface('navigate_to_pose', goal_x=self.current_waypoint[0], goal_y=self.current_waypoint[1], goal_theta=self.current_waypoint[2])

    def execute(self):

        if not self.goal_reached:
            self.is_nav_complete_counter += 1

            if self.is_nav_complete_counter >= 100:
                self.goal_reached = self.action_interface('is_nav_complete')
                self.is_nav_complete_counter = 0

            if self.goal_reached == True:
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    self.current_waypoint = self.waypoints[self.waypoint_index]
                    self.goal_reached = False
                    self.action_interface('navigate_to_pose', goal_x=self.current_waypoint[0], goal_y=self.current_waypoint[1], goal_theta=self.current_waypoint[2])
                else:
                    self.status = "COMPLETED"

    def exit(self):
        self.waypoints = []
        self.current_waypoint = None
        self.waypoint_index = 0
        self.goal_reached = True

    def load_waypoints(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.waypoints = []
                for line in reader:
                    if len(line) == 3:
                        command = (line[0], line[1], line[2])
                        self.waypoints.append(command)
                    else:
                        self.logger.error(f"Malformed line in command file, expected 3 elements but got {len(line)}: {line}")
        except IOError:
            self.logger.error("Failed to read the command file, looking in: " + self.command_file)