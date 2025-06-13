#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time

class TurtleBot3Wrapper:
    def __init__(self):
        """
        Initialize the TurtleBot3 wrapper.
        This class provides a simple interface to control the TurtleBot3.
        """
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('turtlebot3_wrapper', anonymous=True)
        
        # Create publisher for movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Wait for publisher to be ready
        time.sleep(0.5)
        
        # Movement state
        self.is_moving = False
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

    def move(self, linear_vel=0.0, angular_vel=0.0, duration=None):
        """
        Move the robot with specified linear and angular velocities.
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
            duration (float, optional): Duration in seconds. If None, robot will keep moving until stopped.
        """
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        self.is_moving = True
        
        if duration is not None:
            start_time = time.time()
            while (time.time() - start_time) < duration and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
            self.stop()
        else:
            self.cmd_vel_pub.publish(twist)

    def stop(self):
        """Stop the robot's movement."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.is_moving = False

    def get_movement_state(self):
        """
        Get the current movement state of the robot.
        
        Returns:
            dict: Dictionary containing movement state information
        """
        return {
            'is_moving': self.is_moving,
            'linear_velocity': self.current_linear_vel,
            'angular_velocity': self.current_angular_vel
        }

# Example usage
if __name__ == '__main__':
    try:
        # Create wrapper instance
        robot = TurtleBot3Wrapper()
        
        # Example: Move forward for 2 seconds
        print("Moving forward...")
        robot.move(linear_vel=0.2, duration=2.0)
        
        # Example: Turn right for 1 second
        print("Turning right...")
        robot.move(angular_vel=-0.5, duration=1.0)
        
        # Example: Move in a curve
        print("Moving in a curve...")
        robot.move(linear_vel=0.2, angular_vel=0.5, duration=3.0)
        
        # Stop the robot
        print("Stopping...")
        robot.stop()
        
    except rospy.ROSInterruptException:
        pass 