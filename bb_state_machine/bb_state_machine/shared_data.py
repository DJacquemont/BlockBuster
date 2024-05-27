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
        self._duplos_stored = 0        

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

    def update_system_infos(self, battery_level, duplos_stored):
        self._battery_level = battery_level
        self._duplos_stored = duplos_stored