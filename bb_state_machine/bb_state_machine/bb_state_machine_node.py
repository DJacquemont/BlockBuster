import rclpy
from rclpy.node import Node
from bb_state_machine.missions import Mission1, Mission2, Mission3
from bb_state_machine.tools import SharedData, RobotStateMachine
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
import numpy as np


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('data_path', '/src/bb_state_machine/config')

        self.shared_data = SharedData()
        self.shared_data.data_path = self.get_parameter('data_path').get_parameter_value().string_value

        self.get_logger().info(f'Blockbuster State Machine node started with data path: {self.shared_data.data_path}')

        self.state_machine = RobotStateMachine(self.get_logger())
        self.state_machine.add_mission("MISSION_1", Mission1("MISSION_1", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_2", Mission2("MISSION_2", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("MISSION_3", Mission3("MISSION_3", self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.set_mission("MISSION_1")

        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer rate as needed
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_mn', 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, '/storage_servo/commands', 10)

        self.yolov6_sub = self.create_subscription(SpatialDetectionArray, '/color/yolov6_Spatial_detections', self.yolov6_callback, 1)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.detected_objects = {}  
        self.distance_threshold = 0.1
        self.next_object_id = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            
            x = trans.transform.rotation.x
            y = trans.transform.rotation.y
            z = trans.transform.rotation.z
            w = trans.transform.rotation.w
            
            _, _, yaw = euler_from_quaternion([x, y, z, w])
            self.shared_data.update_position(trans.transform.translation.x, trans.transform.translation.y, yaw)
            
        except TransformException as ex:
            # self.get_logger().info(f'Could not transform base_link to map: {ex}')
            pass
        
        self.state_machine.execute()

    def yolov6_callback(self, msg):
        for i, detection in enumerate(msg.detections):
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header.frame_id = "oak_rgb_camera_optical_frame"
            point_in_camera_frame.point.x = detection.position.x
            point_in_camera_frame.point.y = detection.position.y
            point_in_camera_frame.point.z = detection.position.z

            try:
                # Transform the point to the base_link frame
                point_in_base_frame = self.tf_buffer.transform(point_in_camera_frame, "base_link", rclpy.duration.Duration(seconds=1))

                # Create a tuple for the object's position
                object_position = (point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z)

                # Check if the object has been detected before
                for object_id, stored_position in self.detected_objects.items():
                    distance = np.sqrt((stored_position[0] - object_position[0])**2 + (stored_position[1] - object_position[1])**2 + (stored_position[2] - object_position[2])**2)
                    if distance < self.distance_threshold:
                        self.get_logger().info(f'This object (ID: {object_id}) has been detected before.')
                        break
                else:
                    # If it's a new object, store it in the dictionary with a new ID
                    self.detected_objects[self.next_object_id] = object_position
                    self.get_logger().info(f'New object detected (ID: {self.next_object_id}).')
                    self.next_object_id += 1

                self.get_logger().info('POSITION X in base_link frame: ' + str(point_in_base_frame.point.x))
                self.get_logger().info('POSITION Y in base_link frame: ' + str(point_in_base_frame.point.y))
                self.get_logger().info('POSITION Z in base_link frame: ' + str(point_in_base_frame.point.z))

                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point_in_base_frame.point.x
                marker.pose.position.y = point_in_base_frame.point.y
                marker.pose.position.z = point_in_base_frame.point.z
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                # Set the lifetime of the marker to a certain duration (e.g., 5 seconds)
                marker.lifetime = rclpy.duration.Duration(seconds=5).to_msg()

                self.marker_pub.publish(marker)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform oak_rgb_camera_optical_frame to base_link: {ex}')
    
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
        elif action_type == 'log_info':
            self.get_logger().info(kwargs.get('message', ''))

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)

    state_machine_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()