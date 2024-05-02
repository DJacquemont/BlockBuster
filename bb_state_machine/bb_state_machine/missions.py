from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_front_clearing import FrontClearing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.tools import SuperState

class Mission1(SuperState):
    def __init__(self, shared_data, action_interface, logger):
        super().__init__(shared_data, logger)
        self.add_substate("AUTO_NAV_A", AutoNavA(self.shared_data, action_interface, logger))
        self.add_substate("FRONT_CLEARING", FrontClearing(self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav(self.shared_data, action_interface, logger, "test.csv"))
        self.default_substate = "AUTO_NAV_A"

class Mission2(SuperState):
    def __init__(self, shared_data, action_interface, logger):
        super().__init__(shared_data, logger)
        self.add_substate("AUTO_NAV_A", AutoNavA(self.shared_data, action_interface, logger))
        self.add_substate("FRONT_CLEARING", FrontClearing(self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav(self.shared_data, action_interface, logger, "test.csv"))
        self.default_substate = "AUTO_NAV_A"

class Mission3(SuperState):
    def __init__(self, shared_data, action_interface, logger):
        super().__init__(shared_data, logger)
        self.add_substate("AUTO_NAV_A", AutoNavA(self.shared_data, action_interface, logger))
        self.add_substate("FRONT_CLEARING", FrontClearing(self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav(self.shared_data, action_interface, logger, "test.csv"))
        self.default_substate = "MAN_NAV"