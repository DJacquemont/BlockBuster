from abc import ABC, abstractmethod
import csv
import numpy as np

"""
This class is used to store shared data between the different states of the robot.
"""
class SharedData:
    def __init__(self):
        self._data_path = None
        
        self._x = 0
        self._y = 0
        self._theta = 0
        self._pitch = 0

        self._odom_x = 0
        self._odom_y = 0
        self._odom_theta = 0

        self._detection_dict = {}

        self._battery_level = 0
        self._duplos_stored = 0        

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y
    
    @property
    def theta(self):
        return self._theta
    
    @property
    def pitch(self):
        return self._pitch
    
    @property
    def odom_x(self):
        return self._odom_x
    
    @property
    def odom_y(self):
        return self._odom_y
    
    @property
    def odom_theta(self):
        return self._odom_theta
    
    @property
    def data_path(self):
        return self._data_path
    
    @property
    def detection_dict(self):
        return self._detection_dict
    
    @property
    def battery_level(self):
        return self._battery_level
    
    @property
    def duplos_stored(self):
        return self._duplos_stored

    def update_position(self, x, y, theta):
        self._x = x
        self._y = y
        self._theta = theta

    def update_odom_position(self, x, y, theta):
        self._odom_x = x
        self._odom_y = y
        self._odom_theta = theta

    def update_pitch(self, pitch):
        self._pitch = pitch

    def update_data_path(self, data_path):
        self._data_path = data_path

    def update_detection_dict(self, detection_dict):
        self._detection_dict = detection_dict

    def update_system_infos(self, battery_level, duplos_stored):
        self._battery_level = battery_level
        self._duplos_stored = duplos_stored

"""
This class is used to store the different states of the robot.
"""
class RobotStateMachine:
    def __init__(self, logger):
        self.missions = {}
        self.current_mission = None
        self.logger = logger
        self.status = "RUNNING"

    def add_mission(self, name, mission):
        self.missions[name] = mission

    def set_mission(self, name):
        self.logger.info(f"Setting mission: {name}")
        if self.current_mission:
            self.current_mission.exit()
        self.current_mission = self.missions[name]
        self.current_mission.enter()
        if hasattr(self.current_mission, 'default_substate'):
            self.current_mission.set_substate(self.current_mission.default_substate)

    def execute(self):
        if self.current_mission and self.status == "RUNNING":
            self.current_mission.execute()
            if self.current_mission.status == 'COMPLETED':
                next_mission = self.determine_next_state()
                if next_mission:
                    self.set_mission(next_mission)

    def determine_next_state(self):
        
        current_name = self.current_mission.name
        self.status = "COMPLETED"
        self.logger.info("All missions completed.")
        return None
        
        # if current_name == "MISSION_1":
        #     return "MISSION_2"

        # elif current_name == "MISSION_2":
        #     return "MISSION_3"
        
        # else:
        #     self.status = "COMPLETED"
        #     self.logger.info("All missions completed.")
        #     return None


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

    def execute_rotation(self, angle, angular_speed):
        current_theta = self.normalize_angle(self.shared_data.theta)
        target_theta = self.normalize_angle(angle)
        angle_diff = self.angle_difference(target_theta, current_theta)

        target_theta_speed = 0 if abs(angle_diff) < self.angle_tolerance else np.sign(angle_diff) * angular_speed
        self.goal_reached = abs(angle_diff) < self.angle_tolerance

        self.action_interface('publish_cmd_vel', angular_z=target_theta_speed)

    def normalize_angle(self, angle):
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        return angle

    def execute_translation(self, distance, speed):
        current_distance_to_goal = distance - np.sqrt((self.start_pose[0] - self.shared_data.x) ** 2 + (self.start_pose[1] - self.shared_data.y) ** 2)

        target_x_speed = 0 if current_distance_to_goal <= self.distance_tolerance else np.sign(distance) * speed
        self.goal_reached = current_distance_to_goal <= self.distance_tolerance
            
        self.action_interface('publish_cmd_vel', linear_x=target_x_speed)


"""
This class is used to store the different super-states (Missions) of the robot.
"""
class SuperState(BaseState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.substates = {}
        self.current_substate = None
        self.default_substate = None

    def add_substate(self, name, state):
        self.substates[name] = state

    def set_substate(self, name):
        if self.current_substate:
            self.current_substate.exit()
        self.logger.info(f"Current substate (SET SUBSTATE) : {name}")
        self.current_substate = self.substates[name]
        self.current_substate.enter()

    def enter(self):
        self.status = "RUNNING"

    def exit(self):
        pass

    def execute(self):
        if self.current_substate and self.status == "RUNNING":
            self.current_substate.execute()
            if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
                next_state = self.determine_next_state()
                if next_state:
                    self.set_substate(next_state)
