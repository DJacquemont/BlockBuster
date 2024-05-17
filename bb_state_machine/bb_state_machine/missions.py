from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_auto_nav_t import AutoNavT
from bb_state_machine.state_front_clearing import FrontClearing
from bb_state_machine.state_slope_climbing import SlopeClimbing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.tools import SuperState

class Mission1(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/test_auto_nav_t.csv"))
        self.add_substate("AUTO_NAV_A", AutoNavA("AUTO_NAV_A", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test_man_nav.csv"))
        self.default_substate = "MAN_NAV"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            current_ss = self.current_substate.name
            
            if current_ss == "MAN_NAV":
                return "AUTO_NAV_A"

            elif current_ss == "AUTO_NAV_A":
                return "AUTO_NAV_T"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_ss == "AUTO_NAV_T":
                self.status = "COMPLETED"

        return None  # Default to no transition if not specified above
    
class Mission2(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/test_auto_nav_t.csv"))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test_man_nav.csv"))
        self.default_substate = "AUTO_NAV_T"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            current_ss = self.current_substate.name
            
            if current_ss == "AUTO_NAV_T":
                return "AUTO_NAV_A_1"

            elif current_ss == "AUTO_NAV_A_1":
                return "MAN_NAV"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_ss == "MAN_NAV":
                if False: # Check if there are still waypoints left and duplo left
                    return "AUTO_NAV_A_2"
                else:
                    self.status = "COMPLETED"
                    return None
            
            elif current_ss == "AUTO_NAV_A_2":
                    return "AUTO_NAV_T"

        return None  # Default to no transition if not specified above
    
class Mission3(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/test_auto_nav_t.csv"))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test_man_nav.csv"))
        self.default_substate = "AUTO_NAV_A_1"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            current_ss = self.current_substate.name
            
            if current_ss == "AUTO_NAV_T":
                return "AUTO_NAV_A_1"

            elif current_ss == "AUTO_NAV_A_1":
                return "MAN_NAV"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_ss == "MAN_NAV":
                if False:
                    return "AUTO_NAV_A_2"
                else:
                    self.status = "COMPLETED"
                    return None
            
            elif current_ss == "AUTO_NAV_A_2":
                    return "AUTO_NAV_T"

        return None  # Default to no transition if not specified above