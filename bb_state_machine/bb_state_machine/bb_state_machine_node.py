import rclpy
from rclpy.node import Node
from bb_state_machine.missions import Mission1, Mission2, Mission3

class RobotStateMachine:
    def __init__(self):
        self.missions = {}
        self.current_mission = None

    def add_mission(self, name, mission):
        self.missions[name] = mission

    def set_mission(self, name):
        if self.current_mission:
            self.current_mission.exit()
        self.current_mission = self.missions[name]
        self.current_mission.enter()

    def execute(self):
        if self.current_mission:
            self.current_mission.execute()

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.state_machine = RobotStateMachine()
        self.state_machine.add_mission("Mission1", Mission1())
        self.state_machine.add_mission("Mission2", Mission2())
        self.state_machine.add_mission("Mission3", Mission3())
        self.state_machine.set_mission("Mission1")
        self.timer = self.create_timer(1, self.timer_callback)  # adjust timer rate as needed

    def timer_callback(self):
        self.state_machine.execute()

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)

    state_machine_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()