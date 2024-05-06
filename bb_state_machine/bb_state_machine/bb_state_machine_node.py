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
        self.servo_pub = self.create_publisher(Float64MultiArray, '/storage_servo/commands', 10)  # Initialize the publisher
        ...

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
            self.shared_data.update_position(trans.transform.translation.x, trans.transform.translation.y, trans.transform.rotation.z)
            
        except TransformException as ex:
            # self.get_logger().info(f'Could not transform base_link to map: {ex}')
            pass
        
        self.state_machine.execute()
    
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