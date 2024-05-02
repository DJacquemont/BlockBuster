from abc import ABC, abstractmethod
import csv

class SharedData:
    def __init__(self):
        self.data_path = None
        
        self._x = 0
        self._y = 0
        

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def update_position(self, x, y):
        self._x = x
        self._y = y

class RobotStateMachine:
    def __init__(self):
        self.missions = {}
        self.current_mission = None

    def add_mission(self, name, mission):
        self.missions[name] = mission

    def set_mission(self, name):
        if self.current_mission:
            self.current_mission.exit()
        self.current_mission = self.missions[name]
        self.current_mission.enter()
        if hasattr(self.current_mission, 'default_substate'):
            self.current_mission.set_substate(self.current_mission.default_substate)

    def execute(self):
        if self.current_mission:
            self.current_mission.execute()


class BaseState(ABC):
    """
    A base class for all robot states in the state machine.
    """
    def __init__(self, shared_data, action_interface, logger):
        self.shared_data = shared_data
        self.action_interface = action_interface
        self.logger = logger
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
    def __init__(self, shared_data, logger):
        super().__init__(shared_data, None, logger)
        self.substates = {}
        self.current_substate = None
        self.default_substate = None

    def add_substate(self, name, state):
        self.substates[name] = state

    def set_substate(self, name):
        if self.current_substate:
            self.current_substate.exit()
        self.current_substate = self.substates[name]
        self.current_substate.enter()

    def enter(self):
        print("Entering superstate")
        pass  # Optional: Code when entering a superstate

    def execute(self):
        if self.current_substate:
            self.current_substate.execute()

    def exit(self):
        if self.current_substate:
            self.current_substate.exit()  # Optional: Cleanup when exiting a superstate

