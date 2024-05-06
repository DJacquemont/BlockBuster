from bb_state_machine.tools import BaseState

class AutoNavT(BaseState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)

    def enter(self):
        self.logger.info("Entering state: AUTO_NAV_T")
        self.status = "RUNNING"
        # Initialize navigation parameters

    def execute(self):
        self.status = "COMPLETED"
        # Implementation for auto navigation avoiding obstacles

    def exit(self):
        # Clean up or reset navigation parameters
        pass