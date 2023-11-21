from geometry_msgs.msg import Twist
import threading
import time
import datetime as dt


thread = None


class Velocity:
    def __init__(self):
        super().__init__()
        self._linear_velocity = 6.0
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

#            self.spot()

#        def spot(self):
#            if 신호왔는지 확인, 확인완료:
#                   self.stop
#            if self._linear_velocity == 0.0:
#                
#                start_time = dt.datetime.now() 장애물 발견시각
#                time.sleep(3.0)
#                end_time = dt.datetime.now() 3초 지났는지 확인용
#                if(3초 지났으면 end_time - start_time >= 3.0):    
#                   self._linear_velocity = 6.0
#                elif(아직 안지났으면 end_time - start_time <= 3.0) 오차 발생 방지
#                   time.sleep(0.01) 0.01 초씩 슬립하고 재확인
#                   end_time = dt.datetime.now()


def main():
    vel = Velocity()
    for i in range(100):
        print('linear = %f, angular = %f' %
              (vel.linear_velocity, vel.angular_velocity))
        time.sleep(0.5)


if __name__ == '__main__':
    main()
