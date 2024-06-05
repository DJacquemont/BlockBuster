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
        
        if current_ss_name == "GOTO_Z3" and current_ss_status == "COMPLETED":
            if self.shared_data.button_pressed:
                return "SEARCH_Z3"
            else:
                return "PUSH_BUTTON"

        elif current_ss_name == "PUSH_BUTTON" and current_ss_status == "COMPLETED":
            self.shared_data.update_button_pressed()
            return "SEARCH_Z3"

        elif current_ss_name == "SEARCH_Z3" and current_ss_status == "STORAGE_FULL":
            return "HOMING"

        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            return "UNLOADING"

        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.shared_data.duplo_left_z3 <= 0:
                self.status = "COMPLETED"
                return None
            else:
                return "GOTO_Z3"

        elif current_ss_name == "SEARCH_Z3" and current_ss_status == "COMPLETED":
            self.status = "COMPLETED"
            return None
    
class Mission2(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("SEARCH_Z4", AutoNavT("SEARCH_Z4", self.shared_data, action_interface, logger, filename="/m2_search_z4.csv"))
        self.add_substate("GOTO_Z4", AutoNavA("GOTO_Z4", self.shared_data, action_interface, logger, filename="/m2_goto_z4.csv"))
        self.add_substate("SLOPE_UP_1", SlopeClimbing("SLOPE_UP_1", self.shared_data, action_interface, logger, angle_limit=1.51, angular_speed_z = 0.06, direction_up=True))
        self.add_substate("SLOPE_UP_2", SlopeClimbing("SLOPE_UP_2", self.shared_data, action_interface, logger, angle_limit=1.51, angular_speed_z = 0.06, direction_up=False))
        self.add_substate("APPROACH_SLOPE_LOW", ManNav("APPROACH_SLOPE_LOW", self.shared_data, action_interface, logger, filename="/m2_approach_slope_low.csv"))
        self.add_substate("LEAVE_SLOPE_HIGH", ManNav("LEAVE_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/m2_leave_slope_high.csv"))
        self.add_substate("GOTO_SLOPE_HIGH", AutoNavA("GOTO_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/m2_goto_slope_high.csv"))
        self.add_substate("APPROACH_SLOPE_HIGH", ManNav("APPROACH_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/m2_approach_slope_high.csv"))
        self.add_substate("SLOPE_DOWN_1", SlopeClimbing("SLOPE_DOWN_1", self.shared_data, action_interface, logger, angle_limit=1.15, angular_speed_z = -0.1, direction_up=False))
        self.add_substate("SLOPE_DOWN_2", SlopeClimbing("SLOPE_DOWN_2", self.shared_data, action_interface, logger, angle_limit=1.15, angular_speed_z = -0.1, direction_up=True))
        self.add_substate("LEAVE_SLOPE_LOW", ManNav("LEAVE_SLOPE_LOW", self.shared_data, action_interface, logger, filename="/m2_leave_slope_low.csv"))
        self.add_substate("HOMING", AutoNavA("HOMING", self.shared_data, action_interface, logger, filename="/m2_homing.csv"))
        self.add_substate("UNLOADING", ManNav("UNLOADING", self.shared_data, action_interface, logger, filename="/m2_unloading.csv"))
        self.default_substate = "GOTO_Z4"

        self.mission_2_completed = False

    def determine_next_state(self):
        current_ss_name = self.current_substate.name
        current_ss_status = self.current_substate.status

        if current_ss_name == "GOTO_Z4" and current_ss_status == "COMPLETED":
                return "APPROACH_SLOPE_LOW"
        
        elif current_ss_name == "APPROACH_SLOPE_LOW" and current_ss_status == "COMPLETED":
                return "SLOPE_UP_1"
        
        elif current_ss_name == "SLOPE_UP_1" and current_ss_status == "COMPLETED":
                return "SLOPE_UP_2"
        
        elif current_ss_name == "SLOPE_UP_2" and current_ss_status == "COMPLETED":
                return "LEAVE_SLOPE_HIGH"
        
        elif current_ss_name == "LEAVE_SLOPE_HIGH" and current_ss_status == "COMPLETED":
                return "SEARCH_Z4"
        
        elif current_ss_name == "SEARCH_Z4" and current_ss_status == "STORAGE_FULL":
            return "GOTO_SLOPE_HIGH"
        
        elif current_ss_name == "GOTO_SLOPE_HIGH" and current_ss_status == "COMPLETED":
            return "APPROACH_SLOPE_HIGH"
        
        elif current_ss_name == "APPROACH_SLOPE_HIGH" and current_ss_status == "COMPLETED":
            return "SLOPE_DOWN_1"
        
        elif current_ss_name == "SLOPE_DOWN_1" and current_ss_status == "COMPLETED":
            return "SLOPE_DOWN_2"

        elif current_ss_name == "SLOPE_DOWN_2" and current_ss_status == "COMPLETED":
            return "LEAVE_SLOPE_LOW"
        
        elif current_ss_name == "LEAVE_SLOPE_LOW" and current_ss_status == "COMPLETED":
            return "HOMING"
        
        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            return "UNLOADING"
        
        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.shared_data.duplo_left_z4 <= 0 or self.mission_2_completed:
                self.status = "COMPLETED"
                return None
            else:
                return "GOTO_Z4"
        
        elif current_ss_name == "SEARCH_Z4" and current_ss_status == "COMPLETED":
            self.mission_2_completed = True
            return "GOTO_SLOPE_HIGH"
    
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