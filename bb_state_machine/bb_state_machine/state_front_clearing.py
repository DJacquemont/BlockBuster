from bb_state_machine.base_state import BaseState
import numpy as np

class FrontClearing(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, commands=None):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.commands = commands if commands is not None else [(1, 0.2), (-1, 0.2)]
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def enter(self):
        self.logger.info("Entering state: FRONT_CLEARING")
        self.status = "RUNNING"
        self.command_index = 0
        self.goal_reached = False
        self.set_start_pose()
        self.set_current_command()

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if self.status == "RUNNING" and not self.goal_reached:
            self.execute_current_command()

            if self.goal_reached:
                if self.command_index < len(self.commands) - 1:
                    self.command_index += 1
                    self.set_current_command()
                else:
                    self.status = "COMPLETED"

    def reset_navigation_state(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def execute_current_command(self):
        value1, value2 = self.current_command
        self.logger.debug(f"Executing command: t {value1} {value2}")
        self.execute_translation(float(value1), float(value2))

    def set_start_pose(self):
        self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
        if self.start_pose[2] > np.pi:
            self.start_pose[2] -= 2 * np.pi
        self.logger.info(f'Starting pose: {self.start_pose}')

    def set_current_command(self):
        self.current_command = self.commands[self.command_index]
        self.goal_reached = False
        self.set_start_pose()
