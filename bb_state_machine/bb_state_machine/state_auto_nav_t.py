from bb_state_machine.tools import BaseState

class AutoNavT(BaseState):
    def enter(self):
        print("Entering state: AUTO_NAV_T")
        # Initialize navigation parameters

    def execute(self):
        print("Executing AUTO_NAV_T: Navigating while avoiding obstacles with sidequests.")
        # Implementation for auto navigation avoiding obstacles

    def exit(self):
        print("Exiting state: AUTO_NAV_T")
        # Clean up or reset navigation parameters