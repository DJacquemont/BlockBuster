from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_auto_nav_t import AutoNavT
from bb_state_machine.state_front_clearing import FrontClearing
from bb_state_machine.state_slope_climbing import SlopeClimbing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.super_sate import SuperState

class Mission1(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/m1_nav_t_0.csv"))
        self.add_substate("AUTO_NAV_A", AutoNavA("AUTO_NAV_A", self.shared_data, action_interface, logger, filename="/m1_nav_a_0.csv"))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/m1_nav_m_0.csv"))
        self.default_substate = "AUTO_NAV_A"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            current_ss = self.current_substate.name
            
            if current_ss == "AUTO_NAV_A":
                return "MAN_NAV"

            if current_ss == "MAN_NAV":
                return "AUTO_NAV_T"

            elif current_ss == "AUTO_NAV_T":
                self.status = "COMPLETED"

            # self.status = "COMPLETED"

        return None  # Default to no transition if not specified above
    
class Mission2(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/m2_nav_t_0.csv"))
        self.add_substate("AUTO_NAV_A", AutoNavA("AUTO_NAV_A", self.shared_data, action_interface, logger, filename="/m2_nav_a_0.csv"))
        self.add_substate("SLOPE_UP", SlopeClimbing("SLOPE_UP", self.shared_data, action_interface, logger, angle_limit=1.3, direction_up=True))
        self.add_substate("SLOPE_DOWN", SlopeClimbing("SLOPE_DOWN", self.shared_data, action_interface, logger, angle_limit=1.3, direction_up=False, switch_map='l1'))
        # self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test_man_nav.csv"))
        self.default_substate = "AUTO_NAV_A"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            current_ss = self.current_substate.name
            
            if current_ss == "AUTO_NAV_A":
                return "SLOPE_UP"

            elif current_ss == "SLOPE_UP":
                return "SLOPE_DOWN"
            
            elif current_ss == "SLOPE_DOWN":
                return "AUTO_NAV_T"

            elif current_ss == "AUTO_NAV_T":
                    self.status = "COMPLETED"

        return None  # Default to no transition if not specified above
    
class Mission3(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
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