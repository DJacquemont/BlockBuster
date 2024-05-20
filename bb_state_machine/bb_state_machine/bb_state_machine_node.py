import rclpy
from rclpy.node import Node
from bb_state_machine.missions import Mission1, Mission2, Mission3
from bb_state_machine.tools import SharedData, RobotStateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('data_path', '/src/bb_state_machine/config')

        self.shared_data = SharedData()
        self.shared_data.update_data_path(self.get_parameter('data_path').get_parameter_value().string_value)

        self.get_logger().info(f'Blockbuster State Machine node started with data path: {self.shared_data.data_path}')

        self.timer_tf = self.create_timer(0.05, self.timer_tf_callback)
        self.timer_sm_delay = self.create_timer(5.0, self.start_timer_sm)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_mn', 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, '/storage_servo/commands', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.yolov6_sub = self.create_subscription(SpatialDetectionArray, '/color/yolov6_Spatial_detections', self.yolov6_callback, 1, callback_group=self.callback_group)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker', 10)

        self.distance_threshold = 0.1
        self.alpha = 0.2 # EMA coefficient
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.display_marker = True
        self.navigator.waitUntilNav2Active()

        self.state_machine = RobotStateMachine(self.get_logger())
        self.state_machine.add_mission("MISSION_1", Mission1("MISSION_1", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_2", Mission2("MISSION_2", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_3", Mission3("MISSION_3", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.set_mission("MISSION_1")

    def timer_tf_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            _, _, yaw = quaternion_to_euler(trans.transform.rotation.x, 
                                            trans.transform.rotation.y, 
                                            trans.transform.rotation.z, 
                                            trans.transform.rotation.w)
            self.shared_data.update_position(trans.transform.translation.x, trans.transform.translation.y, yaw)
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform base_link to map: {ex}')
            pass
        
    def start_timer_sm(self):
        self.timer_sm = self.create_timer(0.05, self.timer_sm_callback)
        self.timer_sm_delay.cancel()
        self.timer_sm_delay = None 

    def timer_sm_callback(self):        
        self.state_machine.execute()
        self.get_logger().info(f'Duplo dict: {self.shared_data.detection_dict}')

    def imu_callback(self, msg):
        pitch, _, _ = quaternion_to_euler(msg.orientation.x,
                                          msg.orientation.y,
                                          msg.orientation.z,
                                          msg.orientation.w)
        # self.get_logger().info(f'Pitch: {pitch}')
        self.shared_data.update_pitch(pitch)

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
                
                # Only consider x and y coordinates
                object_position = (point_in_base_frame.point.x, point_in_base_frame.point.y)

                already_stored = False
                for object_id, stored_position in detection_dict.items():
                    distance = np.sqrt((stored_position[0] - object_position[0])**2 
                                    + (stored_position[1] - object_position[1])**2)
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
        if action_type == 'publish_cmd_vel':
            msg = Twist()
            msg.linear.x = float(kwargs.get('linear_x', 0))
            msg.angular.z = float(kwargs.get('angular_z', 0))
            self.vel_pub.publish(msg)
        elif action_type == 'publish_servo_cmd':
            servo_command = kwargs.get('servo_command', [0.0])
            msg = Float64MultiArray()
            msg.data = servo_command
            self.servo_pub.publish(msg)
        elif action_type == 'navigate_to_pose':
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(kwargs.get('goal_x', 0))
            goal_pose.pose.position.y = float(kwargs.get('goal_y', 0))
            goal_pose.pose.orientation.z = math.sin(float(kwargs.get('goal_theta', 0)) / 2.0)
            goal_pose.pose.orientation.w = math.cos(float(kwargs.get('goal_theta', 0)) / 2.0)
            self.navigator.goToPose(goal_pose)
        elif action_type == 'is_nav_complete':
            return self.navigator.isTaskComplete()
        elif action_type == 'abort_navigation':
            return self.navigator.cancelTask()
        elif action_type == 'log_info':
            self.get_logger().info(kwargs.get('message', ''))

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

# def main(args=None):
#     rclpy.init(args=args)
#     state_machine_node = StateMachineNode()
#     rclpy.spin(state_machine_node)

#     state_machine_node.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # Add your node to the executor
    executor.add_node(state_machine_node)

    try:
        # Use the executor to spin the node instead of calling rclpy.spin directly
        executor.spin()
    finally:
        # Shutdown and cleanup should be handled in the finally block
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()