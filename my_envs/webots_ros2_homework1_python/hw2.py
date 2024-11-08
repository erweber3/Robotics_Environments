
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class SimpleMove(Node):

    def __init__(self):
        super().__init__('simple_move')
       
        # Declare parameters for speed and distance
        self.declare_parameter('angular_vel', 0.52)  # Default linear velocity
        self.declare_parameter('target_angle', 0.17)  # Default distance to travel
       
        # Get parameters
        self.linear_vel = self.get_parameter('linear_vel').get_parameter_value().double_value
        self.target_angle = self.get_parameter('target_angle').get_parameter_value().double_value
       
        # Initialize distance tracking variables
        self.previous_position = None  # Stores the previous odometry position (x, y)
        self.total_distance_traveled = 0.0  # Keeps track of the total distance
       
        # Publisher for robot velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
       
        # Subscriber to odometry topic to track position
        self.subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            QoSProfile(depth=10))
       
        # Command to be sent
        self.cmd = Twist()
       
        # Start the robot motion timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        # Get the current position of the robot
        current_position = msg.pose.pose.position
       
        # If we have a previous position, calculate the distance traveled
        if self.previous_position is not None:
            distance = self.calculate_distance(self.previous_position, current_position)
            self.total_distance_traveled += distance
       
        # Update the previous position with the current position
        self.previous_position = current_position

        # Log the total distance traveled
        self.get_logger().info(f'Total distance traveled: {self.total_distance_traveled:.2f} meters')

    def calculate_distance(self, previous_position, current_position):
        """Calculates the Euclidean distance between two positions."""
        delta_x = current_position.x - previous_position.x
        delta_y = current_position.y - previous_position.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        return distance

    def timer_callback(self):
        # If the robot has reached or exceeded the target distance, stop
        if self.total_distance_traveled >= self.target_distance:
            self.cmd.linear.x = 0.0  # Stop the robot
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Target distance reached, stopping.')
            self.destroy_node()  # Optionally stop the node after reaching the target
        else:
            # Move forward with the specified linear velocity
            self.cmd.linear.x = self.linear_vel
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Moving forward at {self.linear_vel} m/s')

def main(args=None):
    rclpy.init(args=args)
    simple_move = SimpleMove()  # Instantiate SimpleMove class
    rclpy.spin(simple_move)  # Spin the node
    simple_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
