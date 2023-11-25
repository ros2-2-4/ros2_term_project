import rclpy
import sys
import math
import cv_bridge
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from .velocity import Velocity
from .velocity_calculator import VelocityCalculator
from .line_tracker import LineTracker
from datetime import datetime as dt
from enum import Enum

class StartCarPublisher(Node):
    def __init__(self, vel: Velocity, namespace, line_tracker: LineTracker):
        self.namespace = str(namespace)
        super().__init__('start_car_publisher_' + self.namespace)
        self.vel = vel
        self.line_tracker = line_tracker
        self.start_car_pub = self.create_publisher(String, 'start_car', 10)
        self.timer = self.create_timer(1, self.publish_start_car)
        self.get_logger().info('/start_car topic is published')

    def publish_start_car(self):
        msg = String()
        msg.data = self.namespace
        self.start_car_pub.publish(msg)
        self.get_logger().info('car name : %s is published' % (msg.data))
        velpub = VelPub(self.vel, self.namespace, self.line_tracker)
        rclpy.spin(velpub)        

class VelPub(Node):
    PUB_RATE = 10.0
    DURATION = 1.0

    def __init__(self, vel: Velocity, namespace, line_tracker: LineTracker):
        super().__init__(namespace + '_VelPub')
        self.vel = vel
        self.namespace = namespace
        timer_period = 1 / self.PUB_RATE  # unit time: second
        self.startcar_subscription = self.create_subscription(String, 'start_car', self.start_car_callback, 1)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.target_linear_x = self.vel.linear_velocity        
        self.target_angular_z = self.vel.angular_velocity
        self.linear_x_calculator = VelocityCalculator(1/timer_period, 1.0, 0.0, 0.0)
        self.angular_z_calculator = VelocityCalculator(1/timer_period, 1.0, 0.0, 0.0)

        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self.line_subscription = self.create_subscription(Image, f'{self.namespace}/camera1/image_raw', self.image_callback, 10)
        self.img = None
        
        self.stoptime = None
        self.white_found = False
        self.first_white_detected = False

        self.lidar_subscription = self.create_subscription(LaserScan, f'{self.namespace}/scan', self.scan_callback, 10)

        self.obstacle_found = False
        self.waiting_start_time = None
        self.timechecker = dt.now()

        self.velpub = self.create_publisher(Twist, f'{self.namespace}/cmd_demo', 1)
        self.timer = self.create_timer(timer_period, self.velpub_pub)
        self.msg = Twist()

        

    def start_car_callback(self, msg):
        namespace = msg.data
        velpub = VelPub(self.vel, namespace, self.line_tracker)
        rclpy.spin(velpub)

    def image_callback(self, msg: Image):
        if self.obstacle_found: return
        
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)

        if self.line_tracker.white_detected:
            # White line detected, stop the car
            self.vel.linear_velocity = 0.0
            if not self.white_found:
                self.stoptime = dt.now()
                self.white_found = True
                self.get_logger().info('Found white line, stopping the car.')
                return

        # Check if the car is stopped and has been stopped for more than 3 seconds
        if self.white_found:
            elapsed_time = (dt.now() - self.stoptime).total_seconds()
            if elapsed_time > 3.0:
                # Resume movement after 3 seconds
                self.vel.linear_velocity = 4.5
                self.stoptime = None
                self.white_found = False
                self.get_logger().info('Resuming movement after 3 seconds.')

        if not self.white_found:
            # Reset the stop time when the white line is not detected
            # Update the angular velocity based on the yellow line detection
            self.vel.angular_velocity = (-1) * self.line_tracker.delta / 450  # OK: 40
            # Update the linear velocity based on the angular velocity
            new_linear_velocity = 3.0 if (self.vel.angular_velocity >= 0.070) or (
                    self.vel.angular_velocity <= -0.070) else 5.5
            self.vel.linear_velocity = new_linear_velocity

        # self.vel.angular_velocity = (-1) * self.line_tracker.delta / 450  # OK: 40
        # new_linear_velocity = 2.0 if (self.vel.angular_velocity >= 0.070) or (self.vel.angular_velocity <= -0.070) else 4.5
        # self.vel.linear_velocity = new_linear_velocity

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if not self.obstacle_found and min_distance < 12.0:
            self.vel.stop()
            self.obstacle_found = True
            self.get_logger().info('An obstacle found in %.2f m' % min(msg.ranges))
            if self.waiting_start_time is None:
                # set the start time
                self.waiting_start_time = dt.now()
        if self.obstacle_found:
            if self.waiting_start_time is None: return
                # 1.0 + 1.0 deliberately
            if min_distance > 13.0:
                self.obstacle_found = False
                self.waiting_start_time = None
                return
            elapsed_time = (dt.now() - self.waiting_start_time).total_seconds()
            self.get_logger().debug('elapsed time = %f' % elapsed_time)

    def velpub_pub(self):
        new_linear_x = self.vel.linear_velocity
        if new_linear_x != self.target_linear_x:
            self.target_linear_x = new_linear_x
            self.linear_x_calculator = VelocityCalculator(VelPub.PUB_RATE, VelPub.DURATION, 
                                                          self.current_linear_x, self.target_linear_x)
        msg = Twist()
        self.current_linear_x = self.linear_x_calculator.next_value()
        msg.linear.x = float(self.current_linear_x)
        msg.angular.z = self.vel.angular_velocity
        elapsed_time = (dt.now() - self.timechecker).total_seconds()
        self.get_logger().info('name = %s, linear.x = %.3f, angular.z = %.3f, time= %f' %
                               (self.namespace, msg.linear.x, msg.angular.z, elapsed_time))
        self.velpub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vel = Velocity()
    line_tracker = LineTracker()
    # Check if the 'namespace' argument is provided
    if '--ros-args' in sys.argv:
        for i in range(len(sys.argv)):
            if sys.argv[i] == '-p' and i + 1 < len(sys.argv) and 'namespace' in sys.argv[i + 1]:
                # Extract the value following '-p namespace:='
                namespace_arg = sys.argv[i + 1].split('=')[1]
                start_car_publisher = StartCarPublisher(vel, namespace_arg, line_tracker)
                try:
                    rclpy.spin(start_car_publisher)
                except KeyboardInterrupt:
                    pass
                start_car_publisher.destroy_node()
                rclpy.shutdown()
                break
    else:
        # If '--ros-args' is not used, run with a default namespace
        start_car_publisher = StartCarPublisher(vel, 'demo', line_tracker)
        try:
            rclpy.spin(start_car_publisher)
        except KeyboardInterrupt:
            pass
        start_car_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()