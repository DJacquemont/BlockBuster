from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_auto_nav_t import AutoNavT
from bb_state_machine.state_front_clearing import FrontClearing
from bb_state_machine.state_slope_climbing import SlopeClimbing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.super_sate import SuperState

class Mission1(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("SEARCH_Z3", AutoNavT("SEARCH_Z3", self.shared_data, action_interface, logger, filename="/m1_search_z3.csv"))
        self.add_substate("GOTO_Z3", AutoNavA("GOTO_Z3", self.shared_data, action_interface, logger, filename="/m1_goto_z3.csv"))
        self.add_substate("HOMING", AutoNavA("HOMING", self.shared_data, action_interface, logger, filename="/m1_homing.csv"))
        self.add_substate("PUSH_BUTTON", ManNav("PUSH_BUTTON", self.shared_data, action_interface, logger, filename="/m1_push_button.csv"))
        self.add_substate("UNLOADING", ManNav("UNLOADING", self.shared_data, action_interface, logger, filename="/m1_unloading.csv"))
        self.default_substate = "GOTO_Z3"

    def determine_next_state(self):
        current_ss_name = self.current_substate.name
        current_ss_status = self.current_substate.status

        self.logger.info(f"Current substate: {current_ss_name}, status: {current_ss_status}")
        
        if current_ss_name == "GOTO_Z3" and current_ss_status == "COMPLETED":
            if self.shared_data.button_pressed:
                return "PUSH_BUTTON"
            else:
                return "SEARCH_Z3"

        elif current_ss_name == "PUSH_BUTTON" and current_ss_status == "COMPLETED":
            self.shared_data.update_button_pressed()
            return "SEARCH_Z3"

        elif current_ss_name == "SEARCH_Z3" and current_ss_status == "STORAGE_FULL":
            self.status = "HOMING"

        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            self.status = "UNLOADING"

        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.shared_data.duplo_left_z3 <= 0:
                self.status = "COMPLETED"
            else:
                self.status = "GOTO_Z3"

        elif current_ss_name == "SEARCH_Z3" and current_ss_status == "COMPLETED":
            self.status = "COMPLETED"
    
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
        current_ss = self.current_substate.name
        
        if current_ss == "AUTO_NAV_A":
            return "SLOPE_UP"

        elif current_ss == "SLOPE_UP":
            return "SLOPE_DOWN"
        
        elif current_ss == "SLOPE_DOWN":
            return "AUTO_NAV_T"

        elif current_ss == "AUTO_NAV_T":
                self.status = "COMPLETED"
    
class Mission3(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger, filename="/test_auto_nav_t.csv"))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger, filename="/test_auto_nav_a.csv"))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test_man_nav.csv"))
        self.default_substate = "AUTO_NAV_A_1"

    def determine_next_state(self):
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