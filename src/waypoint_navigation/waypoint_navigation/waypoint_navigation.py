#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
from tf_transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_output), -self.max_output)
        self.previous_error = error
        return output

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_1_x', 0.0),
                ('waypoint_1_y', 0.0),
                ('waypoint_2_x', 0.0),
                ('waypoint_2_y', 0.0),
                ('kp', 0.5),
                ('ki', 0.0),
                ('kd', 0.1),
            ]
        )
        
        # Get parameters
        self.waypoints = [
            [self.get_parameter('waypoint_1_x').value, 
             self.get_parameter('waypoint_1_y').value],
            [self.get_parameter('waypoint_2_x').value,
             self.get_parameter('waypoint_2_y').value]
        ]
        
        self.current_waypoint = 0
        self.position = Point()
        self.yaw = 0.0
        
        # PID Controllers
        self.linear_pid = PIDController(
            self.get_parameter('kp').value,
            self.get_parameter('ki').value,
            self.get_parameter('kd').value,
            max_output=0.5
        )
        
        self.angular_pid = PIDController(
            1.5, 0.0, 0.1,  # Angular PID gains
            max_output=1.5
        )
        
        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.navigate)

    def odom_callback(self, msg):
        # Get position
        self.position = msg.pose.pose.position
        
        # Get yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

    def navigate(self):
        if self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            return

        target = self.waypoints[self.current_waypoint]
        dx = target[0] - self.position.x
        dy = target[1] - self.position.y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        
        # Calculate errors
        angle_error = self.normalize_angle(target_yaw - self.yaw)
        distance_error = distance
        
        # Compute control outputs
        dt = 0.1  # Timer period
        angular_vel = self.angular_pid.compute(angle_error, dt)
        linear_vel = self.linear_pid.compute(distance_error, dt)
        
        # Create and publish Twist message
        twist = Twist()
        if distance > 0.1:  # Distance threshold
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
        else:
            self.current_waypoint += 1
            self.linear_pid.integral = 0.0  # Reset PID on waypoint reach
            
        self.cmd_vel_pub.publish(twist)
        
        if self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached!")

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()