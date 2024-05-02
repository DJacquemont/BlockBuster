from bb_state_machine.tools import BaseState

class FrontClearing(BaseState):
    def enter(self):
        print("Entering state: FRONT_CLEARING")
        # Setup for clearing operations

    def execute(self):
        print("Executing FRONT_CLEARING: Clearing distance in front.")
        # Clearing logic

    def exit(self):
        print("Exiting state: FRONT_CLEARING")
        # Clean up or reset after clearing