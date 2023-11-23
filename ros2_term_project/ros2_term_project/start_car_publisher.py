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
import datetime as dt
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
    PUB_RATE = 30.0
    DURATION = 1

    def __init__(self, vel: Velocity, namespace, line_tracker: LineTracker):
        super().__init__(namespace + '_VelPub')
        self.vel = vel
        self.line_tracker = line_tracker
        self.namespace = namespace
        self.bridge = cv_bridge.CvBridge()
        self.line_subscription = self.create_subscription(Image, f'{self.namespace}/camera1/image_raw', self.image_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, f'{self.namespace}/scan', self.scan_callback, 10)
        self.startcar_subscription = self.create_subscription(String, 'start_car', self.start_car_callback, 1)
        self.velpub = self.create_publisher(Twist, f'{self.namespace}/cmd_demo', 1)
        self.msg = Twist()
        timer_period = 1 / self.PUB_RATE  # unit time: second
        self.timer = self.create_timer(timer_period, self.velpub_pub)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.target_linear_x = self.vel.linear_velocity        
        self.target_angular_z = self.vel.angular_velocity
        self.linear_x_calculator = VelocityCalculator(1/timer_period, 2.0, 0.0, 0.0)
        self.angular_z_calculator = VelocityCalculator(1/timer_period, 2.0, 0.0, 0.0)
        self.img = None
        self.obstacle_found = False
        self.waiting_start_time = None
        self.avoidance_move = False
        self.avoidance_start_time = None
        self.avoidance_state = VelPub.State.WAITING
        self.avoidance_sign = 1
        self.avoidance_start_delta = 0
        

    def start_car_callback(self, msg):
        namespace = msg.data
        velpub = VelPub(self.vel, namespace, self.line_tracker)
        rclpy.spin(velpub)

    def image_callback(self, msg: Image):
        if self.obstacle_found: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)
        self.vel.angular_velocity = (-1) * self.line_tracker.delta / 450  # OK: 40
        # 급커브, 차선이탈방지 속도감소
        new_linear_velocity = 2.0 if (self.vel.angular_velocity >= 0.080) or (self.vel.angular_velocity <= -0.080) else 5.5
        self.vel.linear_velocity = new_linear_velocity

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if not self.obstacle_found and min_distance < 12.0:
            self.vel.stop()
            self.obstacle_found = True
            self.get_logger().info('An obstacle found in %.2f m' % min(msg.ranges))
            if self.waiting_start_time is None:
                # set the start time
                self.waiting_start_time = dt.datetime.now()
        if self.obstacle_found:
            if self.waiting_start_time is None: return
                # 1.0 + 1.0 deliberately
            if self.avoidance_state == VelPub.State.WAITING and min_distance > 13.0:
                self.obstacle_found = False
                self.waiting_start_time = None
                return
            elapsed_time = (dt.datetime.now() - self.waiting_start_time).total_seconds()
            self.get_logger().debug('elapsed time = %f' % elapsed_time)
            if self.avoidance_move is False and elapsed_time > 5.0:
                # self.obstacle_found should be set False after the end of avoidance move
                self.avoidance_state = VelPub.State.STEP_ASIDE
                self.avoidance_start_time = dt.datetime.now() # cf. self.get_clock().now()
                self.avoidance_start_delta = self.line_tracker.delta
                self.avoidance_sign = -1 if self.line_tracker.delta > 0.0 else 1
                self.get_logger().info('delta = %.2f' % self.line_tracker.delta)
                self.avoidance_move = True

        if self.avoidance_move:
            if self.avoidance_state == VelPub.State.STEP_ASIDE:
                self.step_aside()
                elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                if elapsed_time > 3.5:
                    print('State changed ... ')
                    self.avoidance_state = VelPub.State.GO_STRAIGHT
                    self.avoidance_start_time = dt.datetime.now()
            elif self.avoidance_state == VelPub.State.GO_STRAIGHT:
                self.go_straight()
                elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                max_time = 2.5 if abs(self.avoidance_start_delta) > 25 else 4.0
                if elapsed_time > max_time:
                    self.avoidance_state = VelPub.State.STEP_IN
                    self.avoidance_start_time = dt.datetime.now()
                    # self.obstacle_found = False # to activate image_callback()
            elif self.avoidance_state == VelPub.State.STEP_IN:
                self.step_in()
                elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                if elapsed_time > 2.0:
                    self.avoidance_state = VelPub.State.WAITING
                    self.avoidance_start_time = None
                    self.avoidance_move = False
                    self.obstacle_found = False
                    self.waiting_start_time = None

    def step_aside(self):
        self.vel.linear_velocity = 2.0
        self.vel.angular_velocity = self.avoidance_sign * (0.475 if abs(self.avoidance_start_delta) > 25 else 0.3)

    def step_in(self):
        self.vel.linear_velocity = 2.0
        self.vel.angular_velocity = (-1) * self.avoidance_sign * (0.2 if abs(self.avoidance_start_delta) > 25 else 0.25)

    def go_straight(self):
        self.vel.linear_velocity = 5.5
        self.vel.angular_velocity = (-1) * self.avoidance_sign * \
            0.15 if abs(self.avoidance_start_delta) > 25 else (-1) * self.avoidance_sign * 0.3

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
        self.get_logger().info('name = %s, linear.x = %.3f, angular.z = %.3f' %
                               (self.namespace, msg.linear.x, msg.angular.z))
        self.velpub.publish(msg)
    
    class State(Enum):
        WAITING = 0
        STEP_ASIDE = 1
        GO_STRAIGHT = 2
        STEP_IN = 3

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
        start_car_publisher = StartCarPublisher(vel, 'default_namespace', line_tracker)
        try:
            rclpy.spin(start_car_publisher)
        except KeyboardInterrupt:
            pass
        start_car_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()