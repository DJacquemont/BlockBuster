import csv
import numpy as np
from bb_state_machine.base_state import BaseState

class ManNav(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def enter(self):
        self.logger.info("Entering state: MAN_NAV")
        self.status = "RUNNING"
        self.commands = self.load_data(self.command_file,  "commands")
        self.command_index = 0
        if self.commands:
            self.current_command = self.commands[self.command_index]
            self.goal_reached = False
            self.start_pose = self.normalize_pose([self.shared_data.x, self.shared_data.y, self.shared_data.theta])
            self.logger.info(f'Starting pose: {self.start_pose}')
        else:
            self.logger.error("No valid commands found in file: {}".format(self.command_file))
            self.status = "COMPLETED"
            self.reset_navigation_state()

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if not self.goal_reached and self.status == "RUNNING":
            command_type, value1, value2 = self.current_command
            self.logger.debug(f"Executing command: {command_type} {value1} {value2}")
            if command_type == 'r':
                self.execute_rotation(float(value1), float(value2))
            elif command_type == 't':
                self.execute_translation(float(value1), float(value2))
            elif command_type == 's':
                self.command_storage(value1)

            if self.goal_reached:
                if self.command_index < len(self.commands) - 1:
                    self.command_index += 1
                    self.current_command = self.commands[self.command_index]
                    self.goal_reached = False
                    self.start_pose = self.normalize_pose([self.shared_data.x, self.shared_data.y, self.shared_data.theta])
                else:
                    self.status = "COMPLETED"

    def reset_navigation_state(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def command_storage(self, command):
        if command == "c":
            self.action_interface('publish_servo_cmd', servo_command=[0.0])
            self.goal_reached = True
        elif command == "o":
            self.action_interface('publish_servo_cmd', servo_command=[1.0])
            self.goal_reached = True