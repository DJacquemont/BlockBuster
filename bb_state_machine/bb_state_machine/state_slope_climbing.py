from bb_state_machine.tools import BaseState
import numpy as np

class SlopeClimbing(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, speed = 0.2, distance_limit = 3.0, angle_limit=1.3, direction_up=1):
        super().__init__(name, shared_data, logger)
        self.action_interface = action_interface
        self.speed = speed
        self.distance_limit = distance_limit
        self.angle_limit = angle_limit
        self.angle_reached = False
        self.direction_up = direction_up
        self.goal_reached = True
        self.start_pose = None

    def enter(self):
        self.logger.info("Entering state: SLOPE_CLIMB")
        self.status = "RUNNING"
        self.goal_reached = False
        self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
        if self.start_pose[2] > np.pi:
            self.start_pose[2] -= 2 * np.pi
        self.logger.info(f'Starting pose: {self.start_pose}')

    def exit(self):
        self.angle_reached = True
        self.goal_reached = True
        self.start_pose = None

    def execute(self):
        if self.status == "RUNNING":

            if self.direction_up and self.shared_data.pitch > self.angle_limit:
                    self.angle_reached = True
            elif not self.direction_up and self.shared_data.pitch < self.angle_limit:
                    self.angle_reached = True
            else:
                self.execute_translation(float(self.distance_limit), float(self.speed))

            if self.goal_reached or self.angle_reached:
                self.action_interface('publish_cmd_vel', linear_x=0)
                self.status == "COMPLETED"
                return

    """
    Execute a translation command.
    """
    def execute_translation(self, distance, speed):
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