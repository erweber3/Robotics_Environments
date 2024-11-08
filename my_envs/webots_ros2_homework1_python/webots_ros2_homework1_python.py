import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time
import matplotlib.pyplot as plt  # Import Matplotlib

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.55
LIDAR_FOLLOW_DISTANCE = 0.55
LIDAR_ASTRAY_DISTANCE = 1
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

RIGHT_BACK_INDEX = 315
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 180
LEFT_SIDE_INDEX = 90

# Store the position for path recording
file = open('robot_path.txt', 'w')

class WallFollow(Node):

    def __init__(self):
        super().__init__('wall_follow_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.laser_forward = 0
        self.laser_right = 0

        self.previous_laser_forward = None
        self.cmd = Twist()
        self.stall_timer = self.create_timer(5, self.check_stall)  # Check stall every 5 seconds

        self.lidar_stall_threshold = 0.005  # Minimum distance change in LIDAR to detect movement (5 cm)
        self.velocity = 0.0
        self.timer = self.create_timer(0.5, self.timer_callback)  # Main timer for movement

        # Initialize a list to store the robot's path and variables for distance tracking
        self.path_data = []
        self.initial_position = None
        self.max_distance = 0.0

    def listener_callback1(self, msg1):
        # Clean the LIDAR data and store it
        scan = msg1.ranges
        self.scan_cleaned = [3.5 if reading == float('Inf') else (0.0 if math.isnan(reading) else reading) for reading in scan]

        # Track the front LIDAR reading
        self.laser_forward = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        self.laser_right = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.position = (position.x, position.y)

        
        
        # Initialize the initial position
        if self.initial_position is None:
            self.initial_position = (position.x, position.y)
        
        x = position.x
        y = position.y
        file.write(f'{x}, {y}\n')
        

    def check_stall(self):
        """Check for a stall based on LIDAR readings."""
        if self.previous_laser_forward is None or self.previous_laser_right is None:
            # Initialize the previous LIDAR reading on the first check
            self.previous_laser_forward = self.laser_forward
            self.previous_laser_right = self.laser_right
            return
        
        # Calculate the change in the front LIDAR reading
        lidar_change = abs(self.laser_forward - self.previous_laser_forward)
        lidar_change2 = abs(self.laser_right - self.previous_laser_forward)

        self.get_logger().info(f'LIDAR change: {lidar_change:.3f}')

        # Check if the robot is commanded to move forward but the LIDAR readings haven't changed
        if lidar_change < self.lidar_stall_threshold and lidar_change2 < .1:
            self.stall = True
            self.get_logger().info(f'Stall detected based on LIDAR: LIDAR change {lidar_change:.3f} is below the threshold')
        else:
            self.stall = False

        # Update the previous LIDAR reading
        self.previous_laser_forward = self.laser_forward
        self.previous_laser_right = self.laser_right

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        back_right_lidar_min = min(self.scan_cleaned[RIGHT_SIDE_INDEX:RIGHT_BACK_INDEX])

        if self.stall:
            self.get_logger().info('Handling stall...')
            self.cmd.linear.x = -1.2  # Back up
            self.publisher_.publish(self.cmd)
            time.sleep(3)  # Pause for 1 second
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.5  # Turn left 90 degrees
            self.publisher_.publish(self.cmd)
            time.sleep(3.5)  # Pause for 2 seconds
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.stall = False  # Reset stall state
            return

        # Obstacle avoidance logic
        if front_lidar_min < SAFE_STOP_DISTANCE or left_lidar_min < .05:
            if self.turtlebot_moving:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping due to obstacle ahead')
            else:
                self.cmd.linear.x = -0.05
                self.cmd.angular.z = 0.9
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Backing up')
                self.turtlebot_moving = True
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = 1.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Avoiding obstacle, turning')
            self.turtlebot_moving = True
        elif right_lidar_min < LIDAR_ASTRAY_DISTANCE - .2:
            self.cmd.linear.x = .18
            self.cmd.angular.z = -0.15
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Too far from wall, turning right')
            self.turtlebot_moving = True
        elif right_lidar_min < LIDAR_ASTRAY_DISTANCE - .1:
            self.cmd.linear.x = .18
            self.cmd.angular.z = -0.35
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Too far from wall, turning right')
            self.turtlebot_moving = True
        elif back_right_lidar_min < 1.2 and back_right_lidar_min > .02:
            self.cmd.linear.x = 0.01
            self.cmd.angular.z = -0.95
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Cornering')
            self.turtlebot_moving = True
        elif right_lidar_min < LIDAR_ASTRAY_DISTANCE:
            self.cmd.linear.x = .18
            self.cmd.angular.z = -0.5
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Too far from wall, turning right')
            self.turtlebot_moving = True
        else:
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Maintaining wall-following')
            self.turtlebot_moving = True

    def save_path_to_text(self):
        """Save recorded path data to a text file."""
        try:
            # Open a text file to write the coordinates
            with open('robot_path.txt', 'w') as file:
                # Write each (x, y) coordinate pair on a new line
                for x, y in self.path_data:
                    file.write(f'{x}, {y}\n')
            # Log success message
            self.get_logger().info(f'Path data saved to robot_path.txt with max distance {self.max_distance:.2f} meters')
        except Exception as e:
            # Log an error message if something goes wrong
            self.get_logger().error(f'Failed to save path data: {e}')


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    
    # Save the path when shutting down
    wall_follow_node.save_path_to_text()
    wall_follow_node.destroy_node()
    rclpy.shutdown()
