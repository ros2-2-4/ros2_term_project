from geometry_msgs.msg import Twist
import threading
import time
import datetime as dt


thread = None


class Velocity:
    def __init__(self):
        super().__init__()
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0

    @property
    def linear_velocity(self):
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, _linear_velocity):
        self._linear_velocity = _linear_velocity

    @property
    def angular_velocity(self):
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, _angular_velocity):
        self._angular_velocity = _angular_velocity

    def stop(self):
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0

def main():
    vel = Velocity()
    for i in range(100):
        print('linear = %f, angular = %f' %
              (vel.linear_velocity, vel.angular_velocity))
        time.sleep(0.5)


if __name__ == '__main__':
    main()
