from blockbuster_sm.base_state import BaseState

class SlopeClimbing(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, speed=0.3, distance_limit=3.0, angle_limit=0.0, angular_speed_z = 0.0, direction_up=True):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.speed = speed
        self.distance_limit = distance_limit
        self.angle_limit = angle_limit
        self.direction_up = direction_up
        self.goal_reached = False
        self.start_pose = None
        self.angular_speed_z = angular_speed_z

    def enter(self):
        self.logger.info(f"Entering state: {self.name}")
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
                self.logger.info("ANGLE IS REACHED")
                self.stop_motion()
                self.status = "COMPLETED"
                return
            
            self.execute_translation(self.distance_limit, self.speed, angular_speed_z= self.angular_speed_z, use_odom=True)
            if self.goal_reached:
                self.stop_motion()
                self.logger.info("STOPPING MOTION")
                self.status = "FAILED"
                return

    def reset_navigation_state(self):
        self.goal_reached = True
        self.start_pose = None

    def check_angle_reached(self):
        if (self.direction_up and self.shared_data.pitch > self.angle_limit) or \
           (not self.direction_up and self.shared_data.pitch < self.angle_limit):
            return True

    def stop_motion(self):
        self.action_interface('publish_cmd_vel', linear_x=0)
