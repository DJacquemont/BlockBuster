import csv
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
            self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
            if self.start_pose[2] > np.pi:
                self.start_pose[2] -= 2 * np.pi
            self.logger.info(f'Starting pose: {self.start_pose}')

    def exit(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

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
                    self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
                    if self.start_pose[2] > np.pi:
                        self.start_pose[2] -= 2 * np.pi
                else:
                    self.status = "COMPLETED"
                    return
                
    """
    Execute a rotation command.
    """
    def angle_difference(self, target, current):
        diff = (target - current) % (2 * np.pi)
        if diff > np.pi:
            diff -= 2 * np.pi
        return diff

    """
    Load the commands from the command file.
    """
    def load_commands(self):
        try:
            with open(self.command_file, mode='r') as file:
                reader = csv.reader(file)
                self.commands = []
                for line in reader:
                    if len(line) == 2:
                        command = (line[0], line[1], None)
                        self.commands.append(command)
                    elif len(line) == 3:
                        command = (line[0], line[1], line[2])
                        self.commands.append(command)
                    else:
                        self.logger.error(f"Malformed line in command file, expected 2 or 3 elements but got {len(line)}: {line}")
        except IOError:
            self.logger.error("Failed to read the command file, looking in: " + self.command_file)

    """
    Execute a rotation command.
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
    Execute a translation command.
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

    """
    Execute a storage command.
    """
    def command_storage(self, command):
        if command == "c":
            self.action_interface('publish_servo_cmd', servo_command=[0.0])
            self.goal_reached = True
        elif command == "o":
            self.action_interface('publish_servo_cmd', servo_command=[1.0])
            self.goal_reached = True