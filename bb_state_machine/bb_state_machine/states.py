from abc import ABC, abstractmethod

# Base state class
####################################################################################################

class BaseState(ABC):
    """
    A base class for all robot states in the state machine.
    """
    def __init__(self):
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

####################################################################################################

# Specific states for Mission 1
class AutoNavA(BaseState):
    def enter(self):
        print("Entering state: AUTO_NAV_A")
        # Initialize navigation parameters

    def execute(self):
        print("Executing AUTO_NAV_A: Navigating while avoiding obstacles.")
        # Implementation for auto navigation avoiding obstacles

    def exit(self):
        print("Exiting state: AUTO_NAV_A")
        # Clean up or reset navigation parameters

# Specific states for Mission 1
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

class ManNav(BaseState):
    def enter(self):
        print("Entering state: MAN_NAV")
        # Setup for manual navigation

    def execute(self):
        print("Executing MAN_NAV: Manual rotation and inversion.")
        # Manual control logic

    def exit(self):
        print("Exiting state: MAN_NAV")
        # Reset manual controls