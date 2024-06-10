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
        self.min_angular_speed = 0.1
        self.min_linear_speed = 0.1
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

    def execute_rotation(self, angle, max_angular_speed, control=True, use_odom=False):
        if use_odom:
            current_theta = self.normalize_angle(self.shared_data.odom_theta)
        else:
            current_theta = self.normalize_angle(self.shared_data.theta)

        target_theta = self.normalize_angle(angle)
        angle_diff = self.angle_difference(target_theta, current_theta)

        if control:
            angular_speed = np.exp(np.abs(angle_diff)*2-1) * np.abs(max_angular_speed)
            angular_speed = max(min(angular_speed, max_angular_speed), self.min_angular_speed)
        else:
            angular_speed = max_angular_speed

        target_theta_speed = 0 if abs(angle_diff) < self.angle_tolerance else np.sign(angle_diff) * np.abs(angular_speed)
        self.goal_reached = abs(angle_diff) < self.angle_tolerance

        self.action_interface('publish_cmd_vel', angular_z=target_theta_speed)

    def normalize_angle(self, angle):
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        return angle

    def execute_translation(self, distance, max_linear_speed, angular_speed_z = 0.0, use_odom=False):

        if use_odom:
            current_distance_to_goal = np.abs(distance) - np.sqrt((self.start_pose[0] - self.shared_data.odom_x) ** 2 + (self.start_pose[1] - self.shared_data.odom_y) ** 2)
        else:
            current_distance_to_goal = np.abs(distance) - np.sqrt((self.start_pose[0] - self.shared_data.x) ** 2 + (self.start_pose[1] - self.shared_data.y) ** 2)

        linear_speed = np.exp(np.abs(current_distance_to_goal)*2-1) * np.abs(max_linear_speed)
        linear_speed = max(min(linear_speed, max_linear_speed), self.min_linear_speed)

        target_x_speed = 0 if current_distance_to_goal <= self.distance_tolerance else np.sign(distance) * linear_speed
        self.goal_reached = current_distance_to_goal <= self.distance_tolerance
            
        self.action_interface('publish_cmd_vel', linear_x=target_x_speed, angular_z = angular_speed_z)

    def load_data(self, file_path, data_type):
        try:
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                target_list = []
                for line in reader:
                    if line[0].startswith('#'):
                        continue
                    
                    if data_type == 'waypoints_t':
                        if len(line) == 4:
                            target_list.append((float(line[0]), float(line[1]), float(line[2]), int(line[3]), None, None))
                        elif len(line) == 6:
                            target_list.append((float(line[0]), float(line[1]), float(line[2]), int(line[3]), float(line[4]), float(line[5])))
                        else:
                            self.logger.error(f"Malformed line in command file, expected 4 or 6 elements but got {len(line)}: {line}")

                    elif data_type == 'commands':
                        if len(line) == 2:
                            target_list.append((line[0], line[1], None, None))
                        elif len(line) == 3:
                            target_list.append((line[0], line[1], line[2], None))
                        elif len(line) == 4:
                            target_list.append((line[0], line[1], line[2], line[3]))
                        else:
                            self.logger.error(f"Malformed line in command file, expected 2, 3 or 4 elements but got {len(line)}: {line}")

                    elif data_type == 'waypoints_a':
                        if len(line) == 4:
                            try:
                                target_list.append((float(line[0]), float(line[1]), float(line[2]), float(line[3])))
                            except ValueError:
                                self.logger.error(f"Invalid number format in line: {line}")
                        else:
                            self.logger.error(f"Malformed line in command file, expected 3 elements but got {len(line)}: {line}")

                    else:
                        self.logger.error(f"Unknown data type: {data_type}")
                        return
            return target_list

        except IOError:
            self.logger.error(f"Failed to read the command file: {self.command_file}")
        except ValueError as e:
            self.logger.error(f"Failed to parse the command file {self.command_file}: {e}")