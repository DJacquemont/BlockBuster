import csv
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bb_state_machine.tools import BaseState
import numpy as np

class ManNav(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, logger)
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
        self.load_commands()
        self.command_index = 0
        if self.commands:
            self.current_command = self.commands[self.command_index]
            self.goal_reached = False
            self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta]

    def execute(self):
        if not self.goal_reached and self.status == "RUNNING":
            command_type, value1, value2 = self.current_command
            if command_type == 'r':
                self.execute_rotation(float(value1), float(value2))
            elif command_type == 't':
                self.execute_translation(float(value1), float(value2))
            elif command_type == 's':
                self.command_storage(value1)

            # Check if goal is reached, then proceed to next command
            if self.goal_reached:
                if self.command_index < len(self.commands) - 1:
                    self.command_index += 1
                    self.current_command = self.commands[self.command_index]
                    self.goal_reached = False
                    self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta]
                else:
                    self.status = "COMPLETED"

    def exit(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def load_commands(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.commands = []
                for line in reader:
                    if len(line) == 2:
                        # If there's only one element, assume a specific default action
                        # or log it as insufficient data depending on your application's requirements
                        command = (line[0], line[1], None)  # Example: use None for unspecified values
                        self.commands.append(command)
                        self.logger.warning(f"Line with insufficient data treated as a special case: {line}")
                    elif len(line) == 3:
                        command = (line[0], line[1], line[2])
                        self.commands.append(command)
                    else:
                        # Log any line that doesn't meet the expected 1 or 3 elements criteria
                        self.logger.error(f"Malformed line in command file, expected 2 or 3 elements but got {len(line)}: {line}")
        except IOError:
            self.logger.error("Failed to read the command file, looking in: " + self.command_file)

    def execute_rotation(self, angle, angular_speed):
        target_theta_speed = 0

        if angle > 0:
            if self.shared_data.theta % (2 * np.pi) > self.start_pose[2] + angle % (2 * np.pi):
                target_theta_speed = 0
                self.goal_reached = True
            else:
                target_theta_speed = angular_speed
        else:
            if self.shared_data.theta % (2 * np.pi) < self.start_pose[2] + angle % (2 * np.pi):
                target_theta_speed = 0
                self.goal_reached = True
            else:
                target_theta_speed = -angular_speed
                
        
        self.action_interface('publish_cmd_vel', angular_z=target_theta_speed)

        

    def execute_translation(self, distance, speed):
        target_x_speed = 0

        if distance > 0:
            if np.sqrt(self.shared_data.x**2+self.shared_data.y**2) > np.sqrt(self.start_pose[0]**2+self.start_pose[1]**2) + distance:
                target_x_speed = 0
                self.goal_reached = True
            else:
                target_x_speed = speed
            
        else:
            if np.sqrt(self.shared_data.x**2+self.shared_data.y**2) < np.sqrt(self.start_pose[0]**2+self.start_pose[1]**2) + distance:
                target_x_speed = 0
                self.goal_reached = True
            else:
                target_x_speed = -speed
        
        self.action_interface('publish_cmd_vel', linear_x=target_x_speed)

    def command_storage(self, command):
        if command == "c":
            self.action_interface('publish_servo_cmd', 1.0)
            self.goal_reached = True
        elif command == "o":
            self.action_interface('publish_servo_cmd', 0.0)
            self.goal_reached = True