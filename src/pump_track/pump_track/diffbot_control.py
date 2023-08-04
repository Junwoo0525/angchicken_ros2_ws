import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class DiffbotPublisher(Node):
    def __init__(self):
        super().__init__('diffbot_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

        self.subscriber_ = self.create_subscription(
            String,
            '/line_instructions',  # Replace with the actual topic name for receiving commands
            self.command_callback,
            10
        )
        self.get_logger().info("Subscriber created")

        self.command_mapping = {
            '1': 'Go Straight',
            '2': 'Turn Right',
            '3': 'Turn Left',
            '4': 'STOP'
        }

        self.msg = Twist()

    def command_callback(self, msg):
        if msg.data in self.command_mapping:
            command_value = self.command_mapping[msg.data]
            self.get_logger().info("Received command '%s', executing predefined motion: %s" % (msg.data, command_value))
            self.set_predefined_velocity(command_value)
        else:
            self.get_logger().warning("Received an unknown command: %s" % msg.data)

    def set_predefined_velocity(self, mode):
        if mode in self.command_mapping:
            #go staight
            if mode == '1':
                linear_x = 3.0
                linear_y = 0.0
                linear_z = 0.0
                angular_x = 0.0
                angular_y = 0.0
                angular_z = 0.0
                self.publish_velocity(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
            #Turn Right
            elif mode == '2':
                linear_x = 0.0
                linear_y = 0.0
                linear_z = 0.0
                angular_x = 0.0
                angular_y = 0.0
                angular_z = -60.0
                self.publish_velocity(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
            #Turn Left
            elif mode == '3':
                linear_x = 0.0
                linear_y = 0.0
                linear_z = 0.0
                angular_x = 0.0
                angular_y = 0.0
                angular_z = 60.0
                self.publish_velocity(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
            elif mode == '4':
                linear_x = 0.0
                linear_y = 0.0
                linear_z = 0.0
                angular_x = 0.0
                angular_y = 0.0
                angular_z = 0.0
                self.publish_velocity(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
            else:
                self.get_logger().warn("Invalid mode: %s" % mode)
        else:
            self.get_logger().warn("Invalid mode: %s" % mode)

    def publish_velocity(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.msg.linear.x = linear_x
        self.msg.linear.y = linear_y
        self.msg.linear.z = linear_z
        self.msg.angular.x = angular_x
        self.msg.angular.y = angular_y
        self.msg.angular.z = angular_z

        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing Twist command')

    def timer_callback(self):
        pass  # Your original timer_callback implementation here

def main(args=None):
    rclpy.init(args=args)
    diffbot_publisher = DiffbotPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(diffbot_publisher)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        diffbot_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
