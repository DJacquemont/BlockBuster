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

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.get_logger().info('State machine node started')

        self.shared_data = SharedData()

        self.declare_parameter('data_path', '/src/bb_state_machine/config')
        self.shared_data.data_path = self.get_parameter('data_path').get_parameter_value().string_value

        self.get_logger().info(f'State machine node started with data path: {self.shared_data.data_path}')

        self.state_machine = RobotStateMachine()
        self.state_machine.add_mission("Mission1", Mission1(self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("Mission2", Mission2(self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.add_mission("Mission3", Mission3(self.shared_data, self.perform_action, self.get_logger()))
        self.state_machine.set_mission("Mission3")

        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer rate as needed
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        self.state_machine.execute()

        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time())
            
            self.get_logger().info(f'Robot pose: {trans.transform.translation.x}, {trans.transform.translation.y}')

            self.shared_data.update_position(trans.transform.translation.x, trans.transform.translation.y)
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform base_link to map: {ex}')

    def listener_callback(self, msg):
        # Process odometry data here
        return
    
    def perform_action(self, action_type, **kwargs):
        if action_type == 'publish_cmd_vel':
            msg = Twist()
            msg.linear.x = kwargs.get('linear_x', 0)
            msg.angular.z = kwargs.get('angular_z', 0)
            self.vel_pub.publish(msg)
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
