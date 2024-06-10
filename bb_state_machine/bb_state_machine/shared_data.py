import numpy as np

"""
This class is used to store shared data between the different states of the robot.
"""
class SharedData:
    def __init__(self):
        self._data_path = None
        
        self._x = 0
        self._y = 0
        self._theta = 0
        self._pitch = 0

        self._odom_x = 0
        self._odom_y = 0
        self._odom_theta = 0

        self._detection_dict = {}

        self._battery_level = 0

        self._duplos_collected = 0
        self._duplos_stored = 0
        self._max_duplos_stored = 4

        self._duplos_left_z3 = 6
        self._duplos_left_z4 = 6
        self._duplos_left_z1 = 15

        self._current_zone = None
        self._zone_2 = [0.0, -7.5, 1.9, -4.1]
        self._zone_3 = [4.5, -2.5, 7.5, 0.5]
        self._zone_4 = [4.5, -7.5, 7.5, -5.5]

        self._costmap = None

        self._front_distance = None
        self._rear_distance = None

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y
    
    @property
    def theta(self):
        return self._theta
    
    @property
    def pitch(self):
        return self._pitch
    
    @property
    def odom_x(self):
        return self._odom_x
    
    @property
    def odom_y(self):
        return self._odom_y
    
    @property
    def odom_theta(self):
        return self._odom_theta
    
    @property
    def data_path(self):
        return self._data_path
    
    @property
    def detection_dict(self):
        return self._detection_dict
    
    @property
    def battery_level(self):
        return self._battery_level
    
    @property
    def duplos_stored(self):
        return self._duplos_stored
    
    @property
    def duplos_collected(self):
        return self._duplos_collected
    
    @property
    def max_duplos_stored(self):
        return self._max_duplos_stored
    
    @property
    def duplo_left_z1(self):
        return self._duplos_left_z1
    
    @property
    def duplo_left_z3(self):
        return self._duplos_left_z3
    
    @property
    def duplo_left_z4(self):
        return self._duplos_left_z4
    
    @property
    def current_zone(self):
        return self._current_zone
    
    @property
    def zone_2(self):
        return self._zone_2
    
    @property
    def zone_3(self):
        return self._zone_3
    
    @property
    def zone_4(self):
        return self._zone_4
    
    @property
    def costmap(self):
        return self._costmap
    
    @property
    def front_distance(self):
        return self._front_distance
    
    @property
    def rear_distance(self):
        return self._rear_distance

    def update_position(self, x, y, theta):
        self._x = x
        self._y = y
        self._theta = theta

    def update_odom_position(self, x, y, theta):
        self._odom_x = x
        self._odom_y = y
        self._odom_theta = theta

    def update_pitch(self, pitch):
        self._pitch = pitch

    def update_data_path(self, data_path):
        self._data_path = data_path

    def update_detection_dict(self, detection_dict):
        self._detection_dict = detection_dict

    def update_system_infos(self, battery_level, duplos_collected):
        if duplos_collected > self._duplos_collected:
            self._duplos_stored += duplos_collected - self._duplos_collected
            self.update_duplos_left()

        self._battery_level = battery_level
        self._duplos_collected = duplos_collected

    def reset_duplos_stored(self):
        self._duplos_stored = 0

    def update_duplos_left(self):
        if self.current_zone == 'ZONE_1':
            self._duplos_left_z1 -= 1
        elif self.current_zone == 'ZONE_3':
            self._duplos_left_z3 -= 1
        elif self.current_zone == 'ZONE_4':
            self._duplos_left_z4 -= 1

    def update_current_zone(self, zone):
        self._current_zone = zone

    def update_costmap(self, costmap):
        self._costmap = costmap

    def update_front_distance(self, front_distance):
        self._front_distance = front_distance

    def update_rear_distance(self, rear_distance):
        self._rear_distance = rear_distance

    def is_circle_free(self, x, y, r, threshold):
        if self.costmap is None:
            return False

        map_x = int((x - self.costmap.metadata.origin.position.x) / self.costmap.metadata.resolution)
        map_y = int((y - self.costmap.metadata.origin.position.y) / self.costmap.metadata.resolution)

        width = self.costmap.metadata.size_x
        height = self.costmap.metadata.size_y
        data = np.array(self.costmap.data).reshape((height, width))

        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if i**2 + j**2 <= r**2:
                    check_x = map_x + i
                    check_y = map_y + j
                    if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                        return False
                    if data[check_y, check_x] > threshold:
                        return False
        return True