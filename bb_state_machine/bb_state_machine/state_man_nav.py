from bb_state_machine.base_state import BaseState
import time

class ManNav(BaseState):
    def __init__(self, name, shared_data, action_interface, logger, filename):
        super().__init__(name, shared_data, action_interface, logger)
        self.action_interface = action_interface
        self.command_file = shared_data.data_path + filename
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None
        self.wait_start_time = None
        self.wait_duration = None

    def enter(self):
        self.logger.info(f"Entering state: {self.name}")
        self.status = "RUNNING"
        self.commands = self.load_data(self.command_file,  "commands")
        self.logger.info(f"Commands man {self.commands}")
        self.command_index = 0
        if self.commands:
            self.current_command = self.commands[self.command_index]
            self.goal_reached = False
            self.start_pose = self.normalize_pose([self.shared_data.x, self.shared_data.y, self.shared_data.theta])
            self.logger.info(f'Starting pose: {self.start_pose}')
        else:
            self.logger.error("No valid commands found in file: {}".format(self.command_file))
            self.status = "COMPLETED"
            self.reset_navigation_state()

    def exit(self):
        self.reset_navigation_state()

    def execute(self):
        if not self.goal_reached and self.status == "RUNNING":
            command_type, value1, value2, value3 = self.current_command
            self.logger.debug(f"Executing command: {command_type} {value1} {value2} {value3}")
            if command_type == 'r':
                if value3:
                    self.execute_rotation(float(value1), float(value2), control=int(value3))
                else:
                    self.execute_rotation(float(value1), float(value2))
            elif command_type == 't':
                if value3:
                    self.execute_translation(float(value1), float(value2), angular_speed_z=float(value3))
                else:
                    self.execute_translation(float(value1), float(value2))
            elif command_type == 's':
                self.command_storage(value1)
            elif command_type == 'w':
                self.execute_wait(float(value1))
            elif command_type == 'c':
                self.execute_costmap_clear()

            if self.goal_reached:
                if self.command_index < len(self.commands) - 1:
                    self.command_index += 1
                    self.current_command = self.commands[self.command_index]
                    self.goal_reached = False
                    self.start_pose = self.normalize_pose([self.shared_data.x, self.shared_data.y, self.shared_data.theta])
                else:
                    self.status = "COMPLETED"

    def reset_navigation_state(self):
        self.commands = []
        self.current_command = None
        self.command_index = 0
        self.goal_reached = True
        self.start_pose = None

    def command_storage(self, command):
        if command == "c":
            self.action_interface('publish_servo_cmd', servo_command=[1.0])
            self.goal_reached = True
        elif command == "o":
            self.action_interface('publish_servo_cmd', servo_command=[2.0])
            self.goal_reached = True

    def execute_wait(self, wait_time):
        if self.wait_start_time is None:
            self.wait_start_time = time.time()
            self.wait_duration = wait_time

        elapsed_time = time.time() - self.wait_start_time
        if elapsed_time >= self.wait_duration:
            self.goal_reached = True
            self.wait_start_time = None
            self.wait_duration = None

    def execute_costmap_clear(self):
        self.action_interface('clear_costmap')