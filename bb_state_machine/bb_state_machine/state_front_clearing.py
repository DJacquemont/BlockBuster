from bb_state_machine.tools import BaseState
import numpy as np

class FrontClearing(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, commands=None):
        super().__init__(name, shared_data, logger)
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
        self._set_start_pose()
        self._set_current_command()

    def exit(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def execute(self):
        if self.status == "RUNNING" and not self.goal_reached:
            self._execute_current_command()

            if self.goal_reached:
                if self.command_index < len(self.commands) - 1:
                    self.command_index += 1
                    self._set_current_command()
                else:
                    self.status = "COMPLETED"

    def _execute_current_command(self):
        value1, value2 = self.current_command
        self.logger.debug(f"Executing command: t {value1} {value2}")
        self._execute_translation(float(value1), float(value2))

    def _set_start_pose(self):
        self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
        if self.start_pose[2] > np.pi:
            self.start_pose[2] -= 2 * np.pi
        self.logger.info(f'Starting pose: {self.start_pose}')

    def _set_current_command(self):
        self.current_command = self.commands[self.command_index]
        self.goal_reached = False
        self._set_start_pose()

    def _execute_translation(self, distance, speed):
        goal_x = self.start_pose[0] + distance * np.cos(self.start_pose[2])
        goal_y = self.start_pose[1] + distance * np.sin(self.start_pose[2])

        current_distance_to_goal = np.sqrt((goal_x - self.shared_data.x) ** 2 + (goal_y - self.shared_data.y) ** 2)

        self.logger.debug(f"Current pose: {[self.shared_data.x, self.shared_data.y, self.shared_data.theta]}")
        self.logger.debug(f"Current distance to goal: {current_distance_to_goal}")

        distance_tolerance = 0.1
        if current_distance_to_goal <= distance_tolerance:
            self.goal_reached = True
            target_x_speed = 0
        else:
            target_x_speed = np.sign(distance) * speed

        self.action_interface('publish_cmd_vel', linear_x=target_x_speed)
