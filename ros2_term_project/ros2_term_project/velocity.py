
from geometry_msgs.msg import Twist
import threading
import time

thread = None


class Velocity:

    def __init__(self, name):
        global thread
        self.name = name
        self.speed_ = None
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.speed_ = self.Speed()

    @property
    def linear_velocity(self):
        return self.speed_.linear_velocity

    @property
    def angular_velocity(self):
        return self.speed_.angular_velocity

    class Speed:
        def __init__(self):
            super().__init__()
            self._linear_velocity = 6.0
            self._angular_velocity = 0.0

        @property
        def linear_velocity(self) -> float:
            return self._linear_velocity

        @property
        def angular_velocity(self) -> float:
            return self._angular_velocity


def main():
    vel = Velocity('PR001')
    print('thread info:', vel.thread)
    for i in range(100):
        print('name = %s, linear = %f, angular = %f' %
              (vel.name, vel.linear_velocity, vel.angular_velocity))
        time.sleep(0.5)

    vel.thread.join()
    print('thread closed...')
    print('thread info:', vel.thread)


if __name__ == '__main__':
    main()
