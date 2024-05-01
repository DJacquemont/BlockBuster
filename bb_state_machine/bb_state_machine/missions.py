from bb_state_machine.states import BaseState, AutoNavA, FrontClearing, ManNav

####################################################################################################
class SuperState(BaseState):
    def __init__(self):
        self.substates = {}
        self.current_substate = None

    def add_substate(self, name, state):
        self.substates[name] = state

    def set_substate(self, name):
        if self.current_substate:
            self.current_substate.exit()
        self.current_substate = self.substates[name]
        self.current_substate.enter()

    def enter(self):
        pass  # Optional: Code when entering a superstate

    def execute(self):
        if self.current_substate:
            self.current_substate.execute()

    def exit(self):
        if self.current_substate:
            self.current_substate.exit()  # Optional: Cleanup when exiting a superstate

####################################################################################################

class Mission1(SuperState):
    def __init__(self):
        super().__init__()
        self.add_substate("AUTO_NAV_A", AutoNavA())
        self.add_substate("FRONT_CLEARING", FrontClearing())
        self.add_substate("MAN_NAV", ManNav())
        self.set_substate("AUTO_NAV_A")  # Start with the initial state

class Mission2(SuperState):
    def __init__(self, ):
        super().__init__()
        self.add_substate("AUTO_NAV_A", AutoNavA())
        self.add_substate("FRONT_CLEARING", FrontClearing())
        self.add_substate("MAN_NAV", ManNav())
        self.set_substate("AUTO_NAV_A")  # Start with the initial state

class Mission3(SuperState):
    def __init__(self, ):
        super().__init__()
        self.add_substate("AUTO_NAV_A", AutoNavA())
        self.add_substate("FRONT_CLEARING", FrontClearing())
        self.add_substate("MAN_NAV", ManNav())
        self.set_substate("AUTO_NAV_A")  # Start with the initial state