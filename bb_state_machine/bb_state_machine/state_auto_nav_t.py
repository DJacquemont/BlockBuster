from bb_state_machine.base_state import BaseState
import math
import numpy as np
from bb_state_machine.utils import is_point_in_zone

class AutoNavT(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename, zone = 'ZONE_3'):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.target_theta_speed = 0.3
        self.target_x_speed = 0.3
        self.zone = zone
        self.reset_navigation_state()

    def reset_navigation_state(self):
        self.waypoints = []
        self.spin_in_place = []
        self.distance_threshold_wp = []
        self.distance_threshold_duplo = 0.25
        self.tracking = None
        self.tracking_id = None
        self.state = None
        self.rotation_start_time = None
        self.duplo_approach_status = None
        self.goal_reached = None
        self.start_pose = None
        self.target_locked = None
        self.alpha_rotation = None
        self.rotation_accumulated = 0
        self.rotation_target = None
        

    def enter(self):
        self.logger.info(f"Entering state: {self.name}")
        self.status = "RUNNING"
        commands = self.load_data(self.command_file, "waypoints_t")
        self.waypoints = [command[:2] for command in commands]
        self.distance_threshold_wp = [command[2] for command in commands]
        self.spin_in_place = [command[3] for command in commands]

        if self.waypoints:
            self.state = "TRACKING"
            self.goal_reached = False
            self.target_locked = False
        else:
            self.logger.error(f"No valid waypoints found in file: {self.command_file}")
            self.status = "COMPLETED"
            self.reset_navigation_state()

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if self.shared_data.duplos_stored >= self.shared_data.max_duplos_stored or \
           self.shared_data.duplo_left_z3 <= 0:
            self.logger.info("Storage full or no duplos left")
            self.status = "STORAGE_FULL"
            return

        if self.state == "TRACKING":
            self.searching_duplo()
        elif self.state == "ROTATION":
            self.executing_360()
        # elif self.state == "ROTATION":
        #     angle_min = 0
        #     angle_max = 3.15/2
        #     self.executing_rotation_between_angles(angle_min, angle_max)

    def searching_duplo(self):
        # self.logger.info(f'Detection {self.shared_data.detection_dict}')
        # self.logger.info(f'Waypoints {self.waypoints}')
        dist_closest_duplo, i_closest_duplo = self.find_closest_target_dict(self.shared_data.detection_dict)
        dist_closest_waypoint, i_closest_waypoint = self.find_closest_target_list(self.waypoints)

        dist_current_target = self.calculate_current_target_distance()

        if not dist_current_target or dist_current_target > 0.4:
            self.update_tracking_target(dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo)
        else:
            self.handle_waypoint_and_duplo_reach(dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo)

    def find_closest_target_dict(self, targets):
        dist_closest = float('inf')
        i_closest = None
        for i, target in targets.items():
            target_x, target_y = target
            if self.zone == 'ZONE_3':
                if not is_point_in_zone([target_x, target_y], self.shared_data.zone_3):
                    continue
            elif self.zone == 'ZONE_4':
                if not is_point_in_zone([target_x, target_y], self.shared_data.zone_4):
                    continue
            else :
                # TODO: to be implemented
                pass
            
            distance = math.sqrt((self.shared_data.x - target_x) ** 2 + (self.shared_data.y - target_y) ** 2)
            if distance < dist_closest:
                dist_closest = distance
                i_closest = i
        return dist_closest, i_closest

    def find_closest_target_list(self, targets):
        dist_closest = float('inf')
        i_closest = None
        for i, target in enumerate(targets):
            target_x, target_y = target
            if not self.shared_data.is_circle_free(target_x, target_y, 0):
                continue

            distance = math.sqrt((self.shared_data.x - target_x) ** 2 + (self.shared_data.y - target_y) ** 2)
            if distance < dist_closest:
                dist_closest = distance
                i_closest = i
        return dist_closest, i_closest

    def calculate_current_target_distance(self):
        if self.tracking:
            target_x, target_y = self.get_current_target_coordinates()
            distance = math.sqrt((self.shared_data.x - target_x) ** 2 + (self.shared_data.y - target_y) ** 2)
            return distance
        return None

    def get_current_target_coordinates(self):
        if self.tracking == "WP":
            return self.waypoints[self.tracking_id]
        elif self.tracking == "DP":
            return self.shared_data.detection_dict[self.tracking_id]

    def update_tracking_target(self, dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo):
        tracking = self.decide_tracking_target(dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo)
        if tracking:
            self.tracking = tracking
            if tracking == "WP":
                self.tracking_id = i_closest_waypoint
                target_x, target_y = self.waypoints[self.tracking_id]
                # self.logger.info(f"Tracking waypoint: {target_x}, {target_y}")
            else:
                self.tracking_id = i_closest_duplo
                target_x, target_y = self.shared_data.detection_dict[self.tracking_id]
                # self.logger.info(f"Tracking duplo: {target_x}, {target_y}")
            self.action_interface('navigate_to_pose', goal_x=target_x, goal_y=target_y, goal_theta=0)
        else:
            self.logger.info("No waypoints or duplos to track")
            self.status = "COMPLETED"

    def decide_tracking_target(self, dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo):
        if isinstance(i_closest_waypoint, int) and isinstance(i_closest_duplo, int):
            return "DP"
            # return "WP" if dist_closest_waypoint + 1 < dist_closest_duplo else "DP"
        elif isinstance(i_closest_waypoint, int):
            return "WP"
        elif isinstance(i_closest_duplo, int):
            return "DP"
        return None

    def handle_waypoint_and_duplo_reach(self, dist_closest_waypoint, i_closest_waypoint, dist_closest_duplo, i_closest_duplo):
        threshold = 0.4 if i_closest_waypoint is None else self.distance_threshold_wp[i_closest_waypoint]

        if dist_closest_waypoint <= threshold and not self.target_locked:
            self.handle_waypoint_reach(i_closest_waypoint)
        if dist_closest_duplo <= self.distance_threshold_duplo:
            self.handle_duplo_reach(i_closest_duplo)

    def handle_waypoint_reach(self, i_closest_waypoint):
        if self.tracking == "WP":
            assert i_closest_waypoint == self.tracking_id
            self.logger.info(f"Waypoint reached: {self.waypoints[self.tracking_id]}")
            self.action_interface('abort_navigation')
            if self.spin_in_place[self.tracking_id]:
                self.state = "ROTATION"
            self.waypoints.pop(self.tracking_id)
            self.distance_threshold_wp.pop(self.tracking_id)
            self.spin_in_place.pop(self.tracking_id)    
            self.tracking = None
            self.tracking_id = None

        elif self.tracking == "DP":
            self.waypoints.pop(i_closest_waypoint)
            self.distance_threshold_wp.pop(i_closest_waypoint)
            self.spin_in_place.pop(i_closest_waypoint)

    def handle_duplo_reach(self, i_closest_duplo):
        if self.duplo_approach_status is None:
            self.start_duplo_approach(i_closest_duplo)
        elif self.duplo_approach_status == "MAN_ROT":
            self.handle_manual_rotation()
        elif self.duplo_approach_status == "MAN_TRANS":
            self.handle_manual_translation(i_closest_duplo)

    def start_duplo_approach(self, i_closest_duplo):
        self.action_interface('abort_navigation')
        self.duplo_approach_status = "MAN_ROT"
        self.target_locked = True
        self.goal_reached = False
        target_x, target_y = self.shared_data.detection_dict[i_closest_duplo]
        self.alpha_rotation = math.atan2((target_y - self.shared_data.y), (target_x - self.shared_data.x))
        self.start_pose = [self.shared_data.x, self.shared_data.y, self.shared_data.theta % (2 * np.pi)]
        if self.start_pose[2] > np.pi:
            self.start_pose[2] -= 2 * np.pi

    def handle_manual_rotation(self):
        self.execute_rotation(self.alpha_rotation, self.target_theta_speed)
        if self.goal_reached:
            self.duplo_approach_status = "MAN_TRANS"
            self.goal_reached = False

    def handle_manual_translation(self, i_closest_duplo):
        self.execute_translation(0.25, self.target_x_speed)
        if self.goal_reached:
            self.reset_duplo_approach(i_closest_duplo)

    def reset_duplo_approach(self, i_closest_duplo):
        self.duplo_approach_status = None
        self.start_pose = None
        self.target_locked = False
        self.alpha_rotation = None
        assert i_closest_duplo == self.tracking_id
        self.logger.info(f"Duplo reached: {self.shared_data.detection_dict[i_closest_duplo]}")
        del self.shared_data.detection_dict[i_closest_duplo]
        self.tracking = None
        self.tracking_id = None

    def executing_360(self):
        target_increment = np.pi / 3
        if self.rotation_target is None:
            self.rotation_accumulated = 0
            self.rotation_target = (self.shared_data.theta + target_increment) % (2 * np.pi)
            if self.rotation_target > np.pi:
                self.rotation_target -= 2 * np.pi

        if self.goal_reached:
            self.goal_reached = False
            self.rotation_accumulated += target_increment

            if self.rotation_accumulated >= 2 * np.pi - 0.1:
                self.finish_rotation()
                return
            else:
                self.rotation_target = (self.shared_data.theta + target_increment) % (2 * np.pi)
                if self.rotation_target > np.pi:
                    self.rotation_target -= 2 * np.pi

        self.execute_rotation(self.rotation_target, self.target_theta_speed, control=False)

    def finish_rotation(self):
        self.action_interface('publish_cmd_vel', angular_z=0)
        self.state = "TRACKING"
        self.rotation_accumulated = 0
        self.rotation_target = None
        # self.rotation_direction = None

    # def executing_rotation_between_angles(self, angle_min, angle_max):
    #     if self.rotation_target is None:
    #         self.rotation_accumulated = 0
    #         current_angle = self.shared_data.theta % (2 * np.pi)
            
    #         if current_angle > np.pi:
    #             current_angle -= 2 * np.pi

    #         if current_angle < angle_min or current_angle > angle_max:
    #             if abs(current_angle - angle_max) < abs(current_angle - angle_min):
    #                 self.rotation_target = angle_min
    #                 self.rotation_direction = -1  # Counter-clockwise
    #             else:
    #                 self.rotation_target = angle_max
    #                 self.rotation_direction = 1   # Clockwise
    #         else:
    #             if abs(current_angle - angle_max) < abs(current_angle - angle_min):
    #                 self.rotation_target = angle_min
    #                 self.rotation_direction = -1  # Counter-clockwise
    #             else:
    #                 self.rotation_target = angle_max
    #                 self.rotation_direction = 1   # Clockwise

    #     if self.goal_reached:
    #         self.goal_reached = False
    #         current_angle = self.shared_data.theta % (2 * np.pi)
            
    #         if current_angle > np.pi:
    #             current_angle -= 2 * np.pi

    #         if self.rotation_direction == 1:
    #             self.rotation_target = angle_max
    #         else:
    #             self.rotation_target = angle_min

    #         if current_angle >= angle_min and current_angle <= angle_max:
    #             self.finish_rotation()
    #             return

    #     self.execute_rotation(self.rotation_target, self.target_theta_speed * self.rotation_direction, control=False)