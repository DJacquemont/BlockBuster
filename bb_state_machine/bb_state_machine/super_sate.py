from bb_state_machine.base_state import BaseState
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
            self.logger.info(f"Current substate : {self.current_substate.name}")
            self.current_substate.execute()
            if self.current_substate.status != 'RUNNING':
                next_state = self.determine_next_state()
                if next_state:
                    self.set_substate(next_state)
