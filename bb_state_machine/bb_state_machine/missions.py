from bb_state_machine.state_auto_nav_a import AutoNavA
from bb_state_machine.state_auto_nav_t import AutoNavT
from bb_state_machine.state_front_clearing import FrontClearing
from bb_state_machine.state_man_nav import ManNav
from bb_state_machine.tools import SuperState

class Mission1(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test.csv"))
        self.default_substate = "AUTO_NAV_T"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            self.current_substate.name
            current_name = self.current_substate.name
            
            if current_name == "AUTO_NAV_T":
                return "AUTO_NAV_A_1"

            elif current_name == "AUTO_NAV_A_1":
                return "MAN_NAV"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_name == "MAN_NAV":
                if False: # Check if there are still waypoints left and duplo left
                    return "AUTO_NAV_A_2"
                else:
                    self.status = "COMPLETED"
                    return None
            
            elif current_name == "AUTO_NAV_A_2":
                    return "AUTO_NAV_T"

        return None  # Default to no transition if not specified above
    
class Mission2(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test.csv"))
        self.default_substate = "AUTO_NAV_T"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            self.current_substate.name
            current_name = self.current_substate.name
            
            if current_name == "AUTO_NAV_T":
                return "AUTO_NAV_A_1"

            elif current_name == "AUTO_NAV_A_1":
                return "MAN_NAV"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_name == "MAN_NAV":
                if False: # Check if there are still waypoints left and duplo left
                    return "AUTO_NAV_A_2"
                else:
                    self.status = "COMPLETED"
                    return None
            
            elif current_name == "AUTO_NAV_A_2":
                    return "AUTO_NAV_T"

        return None  # Default to no transition if not specified above
    
class Mission3(SuperState):
    def __init__(self, name, shared_data, action_interface, logger):
        super().__init__(name, shared_data, logger)
        self.add_substate("AUTO_NAV_T", AutoNavT("AUTO_NAV_T", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_1", AutoNavA("AUTO_NAV_A_1", self.shared_data, action_interface, logger))
        self.add_substate("AUTO_NAV_A_2", AutoNavA("AUTO_NAV_A_2", self.shared_data, action_interface, logger))
        self.add_substate("MAN_NAV", ManNav("MAN_NAV", self.shared_data, action_interface, logger, filename="/test.csv"))
        self.default_substate = "AUTO_NAV_T"

    def determine_next_state(self):
        if self.current_substate.status == 'COMPLETED' or self.current_substate.status == 'SEARCH_BREAK':
            self.current_substate.name
            current_name = self.current_substate.name
            
            if current_name == "AUTO_NAV_T":
                return "AUTO_NAV_A_1"

            elif current_name == "AUTO_NAV_A_1":
                return "MAN_NAV"  # Proceed to MAN_NAV after AUTO_NAV_A

            elif current_name == "MAN_NAV":
                if False:
                    return "AUTO_NAV_A_2"
                else:
                    self.status = "COMPLETED"
                    return None
            
            elif current_name == "AUTO_NAV_A_2":
                    return "AUTO_NAV_T"

        return None  # Default to no transition if not specified above