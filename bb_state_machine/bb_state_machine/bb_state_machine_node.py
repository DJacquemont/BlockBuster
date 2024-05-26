import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from bb_state_machine.missions import Mission1, Mission2, Mission3
from bb_state_machine.shared_data import SharedData
from bb_state_machine.robot_state_machine import RobotStateMachine
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Empty, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
from depthai_ros_msgs.msg import SpatialDetectionArray
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.srv import LoadMap
import tf2_geometry_msgs
import numpy as np
import math


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self._declare_parameters()
        self._init_shared_data()
        self._init_publishers_and_subscribers()
        self._init_timers()
        self._init_tf_listener()
        self._init_navigator()
        self._init_state_machine()
        self._init_map_service()

        self.distance_threshold = 1.0
        self.alpha = 0.5
        self.display_marker = True
        self.get_logger().info(f'Blockbuster State Machine node started with data path: {self.shared_data.data_path}')

    def _declare_parameters(self):
        self.declare_parameter('data_path', '/src/bb_state_machine/config')
        self.declare_parameter('map_1', '')
        self.declare_parameter('map_2', '')

    def _init_shared_data(self):
        self.shared_data = SharedData()
        self.shared_data.update_data_path(self.get_parameter('data_path').get_parameter_value().string_value)

    def _init_publishers_and_subscribers(self):
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_mn', 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, '/storage_servo/commands', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.sys_info_sub = self.create_subscription(WrenchStamped, '/fts_broadcaster/wrench', self.sys_info_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.yolov6_sub = self.create_subscription(SpatialDetectionArray, '/color/yolov6_Spatial_detections', self.yolov6_callback, 1, callback_group=self.callback_group)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker', 10)

    def _init_timers(self):
        self.timer_tf = self.create_timer(0.05, self.timer_tf_callback)
        self.timer_sm_delay = self.create_timer(5.0, self.start_timer_sm)

    def _init_tf_listener(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def _init_navigator(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def _init_state_machine(self):
        self.state_machine = RobotStateMachine(self.get_logger())
        self.state_machine.add_mission("MISSION_1", Mission1("MISSION_1", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_2", Mission2("MISSION_2", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_3", Mission3("MISSION_3", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.set_mission("MISSION_1")

    def _init_map_service(self):
        self.map_1_url = self.get_parameter('map_1').get_parameter_value().string_value
        self.map_2_url = self.get_parameter('map_2').get_parameter_value().string_value
        self.map_service_client = self.create_client(LoadMap, '/map_server/load_map')
        while not self.map_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /map_server/load_map not available, waiting again...')

    def timer_tf_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            _, _, yaw = quaternion_to_euler(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
            self.shared_data.update_position(trans.transform.translation.x, trans.transform.translation.y, yaw)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform base_link to map: {ex}')
        
    def start_timer_sm(self):
        self.timer_sm = self.create_timer(0.05, self.timer_sm_callback)
        self.timer_sm_delay.cancel()
        self.timer_sm_delay = None 

    def timer_sm_callback(self):        
        self.state_machine.execute()

    def imu_callback(self, msg):
        pitch, _, _ = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.shared_data.update_pitch(pitch)

    def sys_info_callback(self, msg):
        battery_level = msg.wrench.force.x
        duplo_cnt = msg.wrench.torque.z
        self.shared_data.update_system_infos(battery_level, duplo_cnt)

    def odom_callback(self, msg):
        self.shared_data.update_odom_position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z)

    def yolov6_callback(self, msg):
        detection_dict = self.shared_data.detection_dict
        for i, detection in enumerate(msg.detections):
            try:
                point_in_camera_frame = PointStamped()
                point_in_camera_frame.header.frame_id = "oak_rgb_camera_optical_frame"
                point_in_camera_frame.point.x = detection.position.x
                point_in_camera_frame.point.y = detection.position.y
                point_in_camera_frame.point.z = detection.position.z
                point_in_base_frame = self.tf_buffer.transform(point_in_camera_frame, "map", rclpy.duration.Duration(seconds=0.5))
                
                object_position = (point_in_base_frame.point.x, point_in_base_frame.point.y)

                already_stored = False
                for object_id, stored_position in detection_dict.items():
                    distance = np.sqrt((stored_position[0] - object_position[0])**2 + (stored_position[1] - object_position[1])**2)
                    if distance < self.distance_threshold:
                        already_stored = True
                        ema_position = [(self.alpha * object_position[j] + (1 - self.alpha) * stored_position[j]) for j in range(2)]
                        detection_dict[object_id] = tuple(ema_position)
                        break
                
                if not already_stored:
                    detection_dict[self.get_new_detection_id(detection_dict)] = object_position
            
                self.shared_data.update_detection_dict(detection_dict)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform oak_rgb_camera_optical_frame to base_link: {ex}')

        self._publish_markers_if_enabled(detection_dict)

    def _publish_markers_if_enabled(self, detection_dict):
        if self.display_marker:
            marker_array = MarkerArray()
            for object_id, position in detection_dict.items():
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 0.4
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.id = object_id
                marker_array.markers.append(marker)
            
            self.marker_pub.publish(marker_array)

    def get_new_detection_id(self, detection_dict):
        all_ids = list(detection_dict.keys())
        max_id = max(all_ids) if all_ids else -1
        return max_id + 1
    
    def perform_action(self, action_type, **kwargs):
        action_handlers = {
            'publish_cmd_vel': self._publish_cmd_vel,
            'publish_servo_cmd': self._publish_servo_cmd,
            'navigate_to_pose': self._navigate_to_pose,
            'is_nav_complete': self.navigator.isTaskComplete,
            'abort_navigation': self.navigator.cancelTask,
            'set_initial_pose': self._set_initial_pose,
            'log_info': lambda: self.get_logger().info(kwargs.get('message', '')),
            'load_map': self._load_map,
        }

        if action_type in action_handlers:
            return action_handlers[action_type](**kwargs)
        else:
            self.get_logger().error(f"Action type '{action_type}' is not recognized.")

    def _publish_cmd_vel(self, **kwargs):
        msg = Twist()
        msg.linear.x = float(kwargs.get('linear_x', 0))
        msg.angular.z = float(kwargs.get('angular_z', 0))
        self.vel_pub.publish(msg)

    def _publish_servo_cmd(self, **kwargs):
        servo_command = kwargs.get('servo_command', [0.0])
        msg = Float64MultiArray()
        msg.data = servo_command
        self.servo_pub.publish(msg)

    def _navigate_to_pose(self, **kwargs):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(kwargs.get('goal_x', 0))
        goal_pose.pose.position.y = float(kwargs.get('goal_y', 0))
        goal_pose.pose.orientation.z = math.sin(float(kwargs.get('goal_theta', 0)) / 2.0)
        goal_pose.pose.orientation.w = math.cos(float(kwargs.get('goal_theta', 0)) / 2.0)
        self.navigator.goToPose(goal_pose)

    def _set_initial_pose(self, **kwargs):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(kwargs.get('initial_x', 0))
        initial_pose.pose.position.y = float(kwargs.get('initial_y', 0))
        initial_pose.pose.orientation.z = math.sin(float(kwargs.get('initial_theta', 0)) / 2.0)
        initial_pose.pose.orientation.w = math.cos(float(kwargs.get('initial_theta', 0)) / 2.0)
        self.navigator.setInitialPose(initial_pose)

    def _load_map(self, **kwargs):
        map_name = kwargs.get('map_name', '')
        if map_name == 'map_1':
            self.load_map(self.map_1_url)
        elif map_name == 'map_2':
            self.load_map(self.map_2_url)
        else:
            self.get_logger().error(f"Map name '{map_name}' not found in parameters.")

    def load_map(self, map_url):
        request = LoadMap.Request()
        request.map_url = map_url
        future = self.map_service_client.call_async(request)
        future.add_done_callback(self.map_response_callback)

    def map_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Map loaded successfully: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()

    executor = MultiThreadedExecutor()

    executor.add_node(state_machine_node)

    try:
        executor.spin()
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
