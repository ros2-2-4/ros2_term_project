import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .velocity import Velocity
from .velocity_calculator import VelocityCalculator
import math

class StartCarPublisher(Node):
    def __init__(self, vel: Velocity, namespace):
        self.namespace = str(namespace)
        super().__init__('start_car_publisher_' + self.namespace)
        self.vel = vel
        self.start_car_pub = self.create_publisher(String, 'start_car', 10)
        self.timer = self.create_timer(1, self.publish_start_car)
        self.get_logger().info('/start_car topic is published')

    def publish_start_car(self):
        msg = String()
        msg.data = self.namespace
        self.start_car_pub.publish(msg)
        self.get_logger().info('car name : %s is published' % (msg.data))
        velpub = VelPub(self.vel, self.namespace)
        rclpy.spin(velpub)

class VelPub(Node):
    PUB_RATE = 5.0
    DURATION = 2.0

    def __init__(self, vel: Velocity, namespace):
        super().__init__(namespace)
        self.vel = vel
        self.namespace = namespace
        self.subscription = self.create_subscription(String, 'start_car', self.start_car_callback, 1)
        self.velpub = self.create_publisher(Twist, f'{self.namespace}/cmd_demo', 1)
        timer_period = 1 / self.PUB_RATE  # unit time: second
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.current_linear_x = 0.0
        self.target_linear_x = vel.linear_velocity
        self.current_angular_z = 0.0
        self.target_angular_z = vel.angular_velocity
        self.linear_x_calculator = VelocityCalculator(VelPub.PUB_RATE, VelPub.DURATION,
                                                      self.current_linear_x, self.target_linear_x)
        self.angular_z_calculator = VelocityCalculator(VelPub.PUB_RATE, VelPub.DURATION,
                                                       self.current_angular_z, self.target_angular_z)

    def start_car_callback(self, msg):
        namespace = msg.data
        velpub = VelPub(self.vel, namespace)
        rclpy.spin(velpub)

    def pub_callback(self):
        # check if the value changed
        new_linear_x = self.vel.linear_velocity
        new_angular_z = self.vel.angular_velocity
        if new_linear_x != self.target_linear_x:
            self.target_linear_x = new_linear_x
            self.linear_x_calculator.update_parameter(self.current_linear_x, self.target_linear_x)
        if new_angular_z != self.target_angular_z:
            self.target_angular_z = new_angular_z
            self.angular_z_calculator.update_parameter(self.current_angular_z, self.target_angular_z)
        msg = Twist()
        self.current_linear_x = self.linear_x_calculator.next_value()
        self.current_angular_z = self.angular_z_calculator.next_value()
        msg.linear.x = float(self.current_linear_x)
        msg.angular.z = float(math.radians(self.current_angular_z))
        self.get_logger().info('name = %s, linear.x = %.3f, angular.z = %.3f' %
                               (self.namespace, msg.linear.x, msg.angular.z))
        self.velpub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Check if the 'namespace' argument is provided
    if '--ros-args' in sys.argv:
        for i in range(len(sys.argv)):
            if sys.argv[i] == '-p' and i + 1 < len(sys.argv) and 'namespace' in sys.argv[i + 1]:
                # Extract the value following '-p namespace:='
                namespace_arg = sys.argv[i + 1].split('=')[1]
                vel = Velocity()
                start_car_publisher = StartCarPublisher(vel, namespace_arg)
                try:
                    rclpy.spin(start_car_publisher)
                except KeyboardInterrupt:
                    pass
                start_car_publisher.destroy_node()
                rclpy.shutdown()
                break
    else:
        # If '--ros-args' is not used, run with a default namespace
        vel = Velocity()
        start_car_publisher = StartCarPublisher(vel, 'default_namespace')
        try:
            rclpy.spin(start_car_publisher)
        except KeyboardInterrupt:
            pass
        start_car_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    