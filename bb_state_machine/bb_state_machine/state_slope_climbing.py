from bb_state_machine.base_state import BaseState

class SlopeClimbing(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, speed=0.2, distance_limit=3.0, angle_limit=1.3, direction_up=True, switch_map=None):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.speed = speed
        self.distance_limit = distance_limit
        self.angle_limit = angle_limit
        self.direction_up = direction_up
        self.switch_map = switch_map
        self.goal_reached = False
        self.start_pose = None

    def enter(self):
        self.logger.info("Entering state: SLOPE_CLIMB")
        self.status = "RUNNING"
        self.goal_reached = False
        self.start_pose = [self.shared_data.odom_x, self.shared_data.odom_y, self.normalize_angle(self.shared_data.odom_theta)]
        self.logger.info(f'Starting pose: {self.start_pose}')

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if self.status == "RUNNING":
            angle_reached = self.check_angle_reached()
            if angle_reached:
                if self.switch_map == 'l0':
                    self.action_interface('load_map', map_name='map_0')
                    self.action_interface('set_initial_pose')
                elif self.switch_map == 'l1':
                    self.action_interface('load_map', map_name='map_1')
                    self.action_interface('set_initial_pose')

                self.stop_motion()
                self.status = "COMPLETED"
                return

            self.execute_translation(self.distance_limit, self.speed, use_odom=True)
            if self.goal_reached:
                self.stop_motion()
                self.status = "COMPLETED"

    def reset_navigation_state(self):
        self.goal_reached = True
        self.switch_map = False
        self.start_pose = None

    def check_angle_reached(self):
        if (self.direction_up and self.shared_data.pitch > self.angle_limit) or \
           (not self.direction_up and self.shared_data.pitch < self.angle_limit):
            return True

    def stop_motion(self):
        self.action_interface('publish_cmd_vel', linear_x=0)
