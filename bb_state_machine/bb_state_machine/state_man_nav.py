import csv
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bb_state_machine.tools import BaseState
import os

class ManNav(BaseState):
    def __init__(self, shared_data, action_interface, logger, filename):
        super().__init__(shared_data, action_interface, logger)
        self.logger.info('Initialing Mission 1 logic')

        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True

        self.logger.info(f'Command file: {self.command_file}')

    def enter(self):
        self.logger.info("Entering state: MAN_NAV")
        self.load_commands()
        self.command_index = 0
        if self.commands:
            self.current_command = self.commands[self.command_index]
            self.goal_reached = False

    def execute(self):
        self.logger.info("Executing manual nav")
        if not self.goal_reached:
            command_type, value = self.current_command
            if command_type == 'r':
                self.execute_rotation(float(value))
            elif command_type == 't':
                self.execute_translation(float(value))
            elif command_type == 'o':
                self.execute_opening()

            # Check if goal is reached, then proceed to next command
            if self.goal_reached and self.command_index < len(self.commands) - 1:
                self.command_index += 1
                self.current_command = self.commands[self.command_index]
                self.goal_reached = False

    def exit(self):
        print("Exiting state: MAN_NAV")
        self.commands = []  # Clear commands after execution

    def load_commands(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.commands = [(line[0], line[1] if len(line) > 1 else None) for line in reader]
        except IOError:
            print("Failed to read the command file")

    def execute_rotation(self, angle):
        # Assuming a function to publish twist messages
        self.action_interface('publish_cmd_vel', angular_z=angle)
        # Implement checking if rotation goal is reached
        self.goal_reached = True  # Placeholder for actual implementation

    def execute_translation(self, distance):
        # Assuming a function to publish twist messages
        self.action_interface('publish_cmd_vel', linear_x=distance)
        # Implement checking if translation goal is reached
        self.goal_reached = True  # Placeholder for actual implementation

    def execute_opening(self):
        # Assuming a function to publish on a specific topic for door opening
        self.action_interface('publish_opening', message="open")
        self.goal_reached = True  # Opening a door is an instantaneous action

