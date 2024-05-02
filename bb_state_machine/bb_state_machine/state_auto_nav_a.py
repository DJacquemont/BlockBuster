from bb_state_machine.tools import BaseState

class AutoNavA(BaseState):
    def enter(self):
        print("Entering state: AUTO_NAV_A")
        # Initialize navigation parameters

    def execute(self):
        print("Executing AUTO_NAV_A: Navigating while avoiding obstacles.")
        print("Shared data: ", self.shared_data.x, self.shared_data.y)
        # Implementation for auto navigation avoiding obstacles

    def exit(self):
        print("Exiting state: AUTO_NAV_A")
        # Clean up or reset navigation parameters