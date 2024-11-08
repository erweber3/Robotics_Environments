import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import time

class RotateController(Node):

    def __init__(self):
        super().__init__('rotate_controller_node')
       
        # Declare parameters for angular velocity and target angle
        self.declare_parameter('angular_vel', 0.52)  # Default angular velocity in rad/s
        self.declare_parameter('target_angle', 10.0)  # Default target angle in degrees
       
        # Get parameters
        self.angular_vel = self.get_parameter('angular_vel').get_parameter_value().double_value
        self.target_angle_degrees = self.get_parameter('target_angle').get_parameter_value().double_value
       
        # Convert the target angle from degrees to radians
        self.target_angle_radians = math.radians(self.target_angle_degrees)
       
        # Initialize rotation tracking variables
        self.initial_yaw = None  # Stores the initial yaw (orientation)
        self.current_yaw = None  # Tracks the current yaw
        self.total_turned_radians = 0.0  # Accumulates the total turned angle in radians
        self.prev_yaw = None  # Stores the previous yaw for delta calculation
       
        # Publisher for robot velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
       
        # Subscriber to odometry topic to track orientation
        self.subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            QoSProfile(depth=10))
       
        # Command to be sent to the robot
        self.cmd = Twist()
       
        # Start the robot rotation timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        # Extract the orientation from odometry (in quaternion form)
        orientation_q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        self.current_yaw = self.quaternion_to_euler_yaw(orientation_q)

        # If this is the first odometry message, set the initial yaw
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            self.prev_yaw = self.current_yaw  # Initialize previous yaw
            self.get_logger().info(f'Initial yaw set: {math.degrees(self.initial_yaw):.2f} degrees')

        # Accumulate the turned angle
        self.update_total_turned_angle()

    def quaternion_to_euler_yaw(self, orientation_q):
        """Convert quaternion to yaw angle in radians."""
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def update_total_turned_angle(self):
        """Accumulate the total angle turned, handling wrap-around."""
        delta_yaw = self.current_yaw - self.prev_yaw

        # Handle wrap-around cases where yaw crosses -π/π boundary
        if delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi

        # Accumulate the total turned angle
        self.total_turned_radians += delta_yaw
        self.prev_yaw = self.current_yaw  # Update previous yaw

    def calculate_angle_turned(self):
        """Return the total angle turned in radians."""
        return abs(self.total_turned_radians)

    def timer_callback(self):
        # Calculate the angle turned so far
        angle_turned_radians = self.calculate_angle_turned()
        angle_turned_degrees = math.degrees(angle_turned_radians)
       
        # Check if the robot has turned the target angle
        if angle_turned_radians >= self.target_angle_radians:
            self.cmd.angular.z = 0.0  # Stop the robot
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Target angle of {self.target_angle_degrees:.2f} degrees reached. Stopping.')
            time.sleep(1)
            self.get_logger().info(f'Turning at {self.angular_vel:.2f} rad/s. Angle turned: {angle_turned_degrees:.2f} degrees')
            self.destroy_node()  # Optionally stop the node after reaching the target
        else:
            # Rotate with the specified angular velocity
            self.cmd.angular.z = self.angular_vel
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Turning at {self.angular_vel:.2f} rad/s. Angle turned: {angle_turned_degrees:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    rotate_controller = RotateController()
    rclpy.spin(rotate_controller)
    rotate_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
