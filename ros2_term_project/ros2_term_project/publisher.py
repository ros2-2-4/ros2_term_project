import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .velocity import Velocity
from .velocity_calculator import VelocityCalculator
import math


class Publisher(Node):
    PUB_RATE = 10.0
    DURATION = 2.0

    def __init__(self, vel: Velocity):
        super().__init__('publisher')
        self.vel = vel
        self.publisher = self.create_publisher(Twist, 'PR001_start', 1)
        timer_period = 1 / Publisher.PUB_RATE  # unit time: second
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.name = vel.name
        self.current_linear_x = 6.0
        self.target_linear_x = 6.0
        self.current_angular_z = 0.0
        self.target_angular_z = 0.0
        self.linear_x_calculator = VelocityCalculator(Publisher.PUB_RATE, Publisher.DURATION,
                                                      self.current_linear_x, self.target_linear_x)
        self.angular_z_calculator = VelocityCalculator(Publisher.PUB_RATE, Publisher.DURATION,
                                                       self.current_angular_z, self.target_angular_z)

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
                               (self.name, msg.linear.x, msg.angular.z))
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    vel = Velocity('PR001')
    publisher = Publisher(vel)
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    print('name = %s, linear = %f, angular = %f' % (vel.name, vel.linear_velocity, vel.angular_velocity))


if __name__ == '__main__':
    main()
