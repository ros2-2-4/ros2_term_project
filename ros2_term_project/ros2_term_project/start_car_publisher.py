import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .velocity import Velocity
from .velocity_calculator import VelocityCalculator
import math
import cv_bridge
from sensor_msgs.msg import Image
from .line_tracker import LineTracker

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
    DURATION = 2.0

    def __init__(self, vel: Velocity, namespace, line_tracker: LineTracker):
        super().__init__(namespace + '_VelPub')
        self.vel = vel
        self.line_tracker = line_tracker
        self.namespace = namespace
        self.bridge = cv_bridge.CvBridge()
        self._subscription = self.create_subscription(Image, f'{self.namespace}/camera1/image_raw', self.image_callback, 10)
        self.scsub = self.create_subscription(String, 'start_car', self.start_car_callback, 1)
        self.velpub = self.create_publisher(Twist, f'{self.namespace}/cmd_demo', 1)
        self.msg = Twist()
        timer_period = 1 / self.PUB_RATE  # unit time: second
        self.timer = self.create_timer(timer_period, self.velpub_pub)
        self.current_linear_x = 0.0
        self.target_linear_x = self.vel.linear_velocity
        self.current_angular_z = 0.0
        self.target_angular_z = self.vel.angular_velocity
        self.linear_x_calculator = VelocityCalculator(1/timer_period, 2.0, 0.0, 0.0)
        self.angular_z_calculator = VelocityCalculator(1/timer_period, 2.0, 0.0, 0.0)
        self.img = None

    def start_car_callback(self, msg):
        namespace = msg.data
        velpub = VelPub(self.vel, namespace, self.line_tracker)
        rclpy.spin(velpub)

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)
        self.vel.angular_velocity = (-1) * self.line_tracker.delta / 450  # OK: 40
        new_linear_velocity = 2.0 if (self.vel.angular_velocity >= 0.070) or (self.vel.angular_velocity <= -0.070) else 5.5
        self.vel.linear_velocity = new_linear_velocity

    def velpub_pub(self):
        # check if the value changed
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