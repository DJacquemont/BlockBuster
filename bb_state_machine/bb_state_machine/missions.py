from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_auto_nav_t import AutoNavT
from bb_state_machine.state_slope_climbing import SlopeClimbing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.super_sate import SuperState

class Mission1(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("SEARCH_Z3", AutoNavT("SEARCH_Z3", self.shared_data, action_interface, logger, filename="/mission1/m1_search_z3.csv", zone='ZONE_3'))
        self.add_substate("SEARCH_Z1", AutoNavT("SEARCH_Z1", self.shared_data, action_interface, logger, filename="/mission1/m1_search_z1.csv", zone='ZONE_1'))
        self.add_substate("GOTO_Z3", AutoNavA("GOTO_Z3", self.shared_data, action_interface, logger, filename="/mission1/m1_goto_z3.csv"))
        self.add_substate("GOBACKTO_Z3", AutoNavA("GOBACKTO_Z3", self.shared_data, action_interface, logger, filename="/mission1/m1_gobackto_z3.csv"))
        self.add_substate("HOMING", AutoNavA("HOMING", self.shared_data, action_interface, logger, filename="/mission1/m1_homing.csv"))
        self.add_substate("OUT_Z3", AutoNavA("OUT_Z3", self.shared_data, action_interface, logger, filename="/mission1/m1_out_z3.csv"))
        self.add_substate("OUT_Z3_MAN", ManNav("OUT_Z3_MAN", self.shared_data, action_interface, logger, filename="/mission1/m1_out_z3_man.csv"))
        self.add_substate("PUSH_BUTTON", ManNav("PUSH_BUTTON", self.shared_data, action_interface, logger, use_odom=False, filename="/mission1/m1_push_button.csv"))
        self.add_substate("REATTEMPT_BUTTON", ManNav("REATTEMPT_BUTTON", self.shared_data, action_interface, logger, filename="/mission1/m1_reattempt_button.csv"))
        self.add_substate("UNLOADING", ManNav("UNLOADING", self.shared_data, action_interface, logger, filename="/general/unloading.csv"))
        self.default_substate = "GOTO_Z3"

        self.zone_3_completed = False
        self.time_is_up = False
        self.zone_3_nb_explore = 0

    def determine_next_state(self):
        current_ss_name = self.current_substate.name
        current_ss_status = self.current_substate.status
        
        if current_ss_name == "GOTO_Z3" and current_ss_status == "COMPLETED":
            return "PUSH_BUTTON"

        elif current_ss_name == "PUSH_BUTTON" and current_ss_status == "COMPLETED":
            if self.shared_data.front_distance > 3:
                return "SEARCH_Z3"
            else:
                return "REATTEMPT_BUTTON"
        
        elif current_ss_name == "REATTEMPT_BUTTON" and current_ss_status == "COMPLETED":
            if self.shared_data.front_distance > 3:
                return "SEARCH_Z3"
            else:
                self.status = "COMPLETED"
                return None

        elif current_ss_name == "SEARCH_Z3":
            self.logger.info(f"current_ss_name {current_ss_name}, current_ss_status {current_ss_status}")
            self.logger.info(f"self.shared_data.duplos_stored {self.shared_data.duplos_stored}")
            self.logger.info(f"self.shared_data.duplo_left_z3 {self.shared_data.duplo_left_z3}")
            if current_ss_status == "STORAGE_FULL":
                self.zone_3_nb_explore += 1
                return "OUT_Z3"
            elif current_ss_status == "COMPLETED":
                self.zone_3_nb_explore += 1
                self.zone_3_completed = True
                return "OUT_Z3"
            
        elif current_ss_name == "OUT_Z3" and current_ss_status == "COMPLETED":
            return "OUT_Z3_MAN"
                
        elif current_ss_name == "OUT_Z3_MAN" and current_ss_status == "COMPLETED":
            if self.shared_data.duplos_stored < self.shared_data.max_duplos_stored and \
                self.shared_data.duplos_stored != 0:
                return "SEARCH_Z1"
            elif self.shared_data.duplos_stored == 0:
                self.status = "COMPLETED"
                return None
            else:
                return "HOMING"

        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            return "UNLOADING"

        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.shared_data.duplo_left_z3 <= 1 or self.zone_3_completed or self.zone_3_nb_explore >= 2:
                self.status = "COMPLETED"
                return None
            else:
                return "GOBACKTO_Z3"
            
        elif current_ss_name == "GOBACKTO_Z3" and current_ss_status == "COMPLETED":
            return "SEARCH_Z3"        
            
        elif current_ss_name == "SEARCH_Z1" and (current_ss_status == "STORAGE_FULL" or current_ss_status == "COMPLETED"):
            return "HOMING"

    
class Mission2(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, action_interface, logger)
        self.add_substate("SEARCH_Z4", AutoNavT("SEARCH_Z4", self.shared_data, action_interface, logger, filename="/mission2/m2_search_z4.csv", zone='ZONE_4'))
        self.add_substate("SEARCH_Z1", AutoNavT("SEARCH_Z1", self.shared_data, action_interface, logger, filename="/mission2/m2_search_z1.csv", zone='ZONE_1'))
        self.add_substate("GOTO_Z4", AutoNavA("GOTO_Z4", self.shared_data, action_interface, logger, filename="/mission2/m2_goto_z4.csv"))
        self.add_substate("SLOPE_UP_1", SlopeClimbing("SLOPE_UP_1", self.shared_data, action_interface, logger, speed=0.5, angle_limit=1.5, angular_speed_z = 0.08, direction_up=True))
        self.add_substate("SLOPE_UP_2", SlopeClimbing("SLOPE_UP_2", self.shared_data, action_interface, logger, speed=0.5, angle_limit=1.5, angular_speed_z = 0.08, direction_up=False))
        self.add_substate("RECOVERY_BEHAVIOR", ManNav("RECOVERY_BEHAVIOR", self.shared_data, action_interface, logger, use_odom = True, filename="/mission2/m2_recovery_behavior.csv"))
        self.add_substate("CALIBRATING", ManNav("CALIBRATING", self.shared_data, action_interface, logger, use_odom = True, filename="/mission2/m2_calibrating.csv"))
        self.add_substate("APPROACH_SLOPE_LOW", ManNav("APPROACH_SLOPE_LOW", self.shared_data, action_interface, logger, filename="/mission2/m2_approach_slope_low.csv"))
        self.add_substate("LEAVE_SLOPE_HIGH", ManNav("LEAVE_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/mission2/m2_leave_slope_high.csv"))
        self.add_substate("GOTO_SLOPE_HIGH", AutoNavA("GOTO_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/mission2/m2_goto_slope_high.csv"))
        self.add_substate("APPROACH_SLOPE_HIGH", ManNav("APPROACH_SLOPE_HIGH", self.shared_data, action_interface, logger, filename="/mission2/m2_approach_slope_high.csv"))
        self.add_substate("SLOPE_DOWN_1", SlopeClimbing("SLOPE_DOWN_1", self.shared_data, action_interface, logger, speed=0.2, distance_limit=3.0, angle_limit=1.2, angular_speed_z = -0.03, direction_up=False))
        self.add_substate("SLOPE_DOWN_2", SlopeClimbing("SLOPE_DOWN_2", self.shared_data, action_interface, logger, speed=0.25, distance_limit=2.0, angle_limit=1.2, angular_speed_z = -0.07, direction_up=True))
        self.add_substate("LEAVE_SLOPE_LOW", ManNav("LEAVE_SLOPE_LOW", self.shared_data, action_interface, logger, filename="/mission2/m2_leave_slope_low.csv"))
        self.add_substate("HOMING", AutoNavA("HOMING", self.shared_data, action_interface, logger, filename="/mission2/m2_homing.csv"))
        self.add_substate("UNLOADING", ManNav("UNLOADING", self.shared_data, action_interface, logger, filename="/general/unloading.csv"))
        self.default_substate = "GOTO_Z4"

        self.mission_2_completed = False
        self.recovery_behavior = False

    def determine_next_state(self):
        current_ss_name = self.current_substate.name
        current_ss_status = self.current_substate.status

        self.logger.info(f"current_ss_name {current_ss_name}, current_ss_status {current_ss_status}")
        self.logger.info(f"self.shared_data.duplo_left_z3 {self.shared_data.duplo_left_z4}")

        if current_ss_name == "GOTO_Z4" and current_ss_status == "COMPLETED":
            if self.shared_data.duplos_stored >= 3:
                return "SEARCH_Z1"
            else:
                return "APPROACH_SLOPE_LOW"
        
        elif current_ss_name == "APPROACH_SLOPE_LOW" and current_ss_status == "COMPLETED":
            return "SLOPE_UP_1"
        
        elif current_ss_name == "SLOPE_UP_1":
            if current_ss_status == "COMPLETED":
                return "SLOPE_UP_2"
            elif current_ss_status == "FAILED":
                return "RECOVERY_BEHAVIOR"
        
        elif current_ss_name == "SLOPE_UP_2":
            if current_ss_status == "COMPLETED":
                return "LEAVE_SLOPE_HIGH"
            elif current_ss_status == "FAILED":
                return "RECOVERY_BEHAVIOR"
        
        elif current_ss_name == "RECOVERY_BEHAVIOR" and current_ss_status == "COMPLETED":
            if not self.recovery_behavior:
                return "CALIBRATING"
            else:
                self.status = "COMPLETED"
                return None
        
        elif current_ss_name == "CALIBRATING" and current_ss_status == "COMPLETED":
            self.recovery_behavior = True
            return "GOTO_Z4"
        
        elif current_ss_name == "LEAVE_SLOPE_HIGH" and current_ss_status == "COMPLETED":
            return "SEARCH_Z4"
        
        elif current_ss_name == "SEARCH_Z4" and current_ss_status == "STORAGE_FULL":
            return "GOTO_SLOPE_HIGH"
        
        elif current_ss_name == "GOTO_SLOPE_HIGH" and current_ss_status == "COMPLETED":
            return "APPROACH_SLOPE_HIGH"
        
        elif current_ss_name == "APPROACH_SLOPE_HIGH" and current_ss_status == "COMPLETED":
            return "SLOPE_DOWN_1"
        
        elif current_ss_name == "SLOPE_DOWN_1":
            if current_ss_status == "COMPLETED":
                return "SLOPE_DOWN_2"
            elif current_ss_status == "FAILED":
                return "LEAVE_SLOPE_LOW"

        elif current_ss_name == "SLOPE_DOWN_2" and (current_ss_status == "COMPLETED" or current_ss_status == "FAILED"):
            return "LEAVE_SLOPE_LOW"
        
        elif current_ss_name == "LEAVE_SLOPE_LOW" and current_ss_status == "COMPLETED":
            if self.shared_data.duplos_stored < self.shared_data.max_duplos_stored and \
                self.shared_data.duplos_stored != 0:
                return "SEARCH_Z1"
            elif self.shared_data.duplos_stored == 0:
                self.status = "COMPLETED"
                return None
            else:
                return "HOMING"
            
        elif current_ss_name == "SEARCH_Z1" and (current_ss_status == "STORAGE_FULL" or current_ss_status == "COMPLETED"):
            return "HOMING"
        
        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            return "UNLOADING"
        
        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.shared_data.duplo_left_z4 <= 2 or self.mission_2_completed:
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
        self.add_substate("SEARCH_Z1", AutoNavT("SEARCH_Z1", self.shared_data, action_interface, logger, filename="/mission3/m3_search_z1.csv", zone='ZONE_1'))
        self.add_substate("HOMING", AutoNavA("HOMING", self.shared_data, action_interface, logger, filename="/mission3/m3_homing.csv"))
        self.add_substate("UNLOADING", ManNav("UNLOADING", self.shared_data, action_interface, logger, filename="/general/unloading.csv"))
        self.default_substate = "SEARCH_Z1"

        self.mission_3_completed = False

    def execute(self):
        if self.current_substate and self.status == "RUNNING":

            if self.shared_data.delta_time > 550 and \
                self.current_substate.current_ss_name == "SEARCH_Z1":
                    self.set_substate("HOMING")

            self.current_substate.execute()
            if self.current_substate.status != 'RUNNING':
                next_state = self.determine_next_state()
                if next_state:
                    self.set_substate(next_state)


    def determine_next_state(self):
        current_ss_name = self.current_substate.name
        current_ss_status = self.current_substate.status
    
        if current_ss_name == "SEARCH_Z1":
            if current_ss_status == "STORAGE_FULL":
                return "HOMING"
            elif current_ss_status == "COMPLETED":
                self.mission_3_completed = True
                if self.shared_data.duplos_stored == 0:
                    self.status = "COMPLETED"
                    return None
                else:
                    return "HOMING"
                
        elif current_ss_name == "HOMING" and current_ss_status == "COMPLETED":
            return "UNLOADING"
        
        elif current_ss_name == "UNLOADING" and current_ss_status == "COMPLETED":
            if self.mission_3_completed:
                self.status = "COMPLETED"
                return None
            else:
                return "SEARCH_Z1"
                