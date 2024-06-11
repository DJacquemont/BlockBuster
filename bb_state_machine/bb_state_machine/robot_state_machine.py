"""
This class is used to store the different states of the robot.
"""
class RobotStateMachine:
    def __init__(self, logger, shared_data):
        self.missions = {}
        self.current_mission = None
        self.logger = logger
        self.shared_data = shared_data
        self.status = "RUNNING"

    def add_mission(self, name, mission):
        self.missions[name] = mission

    def set_mission(self, name):
        self.logger.info(f"Setting mission: {name}")
        if self.current_mission:
            self.current_mission.exit()
        self.current_mission = self.missions[name]
        self.current_mission.enter()
        if hasattr(self.current_mission, 'default_substate'):
            self.current_mission.set_substate(self.current_mission.default_substate)

    def execute(self):
        if self.current_mission and self.status == "RUNNING":
            self.current_mission.execute()
            if self.current_mission.status == 'COMPLETED':
                next_mission = self.determine_next_state()
                if next_mission:
                    self.set_mission(next_mission)

    def determine_next_state(self):
        
        current_name = self.current_mission.name
        
        # if current_name == "MISSION_1":
        #     return "MISSION_3"
        
        # elif current_name == "MISSION_2":
        #     return "MISSION_4"

        # elif current_name == "MISSION_3":
        #     if self.shared_data.delta_time < 480:
        #         return "MISSION_2"
        #     else:
        #         return "MISSION_4"

        if current_name == "MISSION_1":
            self.shared_data.set_max_duplos_stored(3)
            return "MISSION_2"
        
        elif current_name == "MISSION_2":
            self.shared_data.set_max_duplos_stored(5)
            return "MISSION_3"
        
        elif current_name == "MISSION_3":
            self.status = "COMPLETED"
            self.logger.info("All missions completed.")
            return None