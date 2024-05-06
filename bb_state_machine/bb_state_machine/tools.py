from abc import ABC, abstractmethod
import csv

class SharedData:
    def __init__(self):
        self.data_path = None
        
        self._x = 0
        self._y = 0
        self._theta = 0
        

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y
    
    @property
    def theta(self):
        return self._theta

    def update_position(self, x, y, theta):
        self._x = x
        self._y = y
        self._theta = theta

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
        
        if current_name == "MISSION_1":
            return "MISSION_2"

        elif current_name == "MISSION_2":
            return "MISSION_3"
        
        else:
            self.status = "COMPLETED"
            self.logger.info("All missions completed.")
            return None


class BaseState(ABC):
    """
    A base class for all robot states in the state machine.
    """
    def __init__(self, name, shared_data, logger):
        self.name = name
        self.shared_data = shared_data
        self.logger = logger
        self.status = "IDLE"
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


class SuperState(BaseState):
    def __init__(self, name, shared_data, logger):
        super().__init__(name, shared_data, logger)
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

    def execute(self):
        if self.current_substate and self.status == "RUNNING":
            self.current_substate.execute()
            if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
                next_state = self.determine_next_state()
                if next_state:
                    self.set_substate(next_state)

    def exit(self):
        pass

