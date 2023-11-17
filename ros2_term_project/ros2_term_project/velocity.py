
from geometry_msgs.msg import Twist
import threading
import time
import datetime as dt

thread = None


class Velocity:

    def __init__(self):
        global thread
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
#            self.spot()

#        def spot(self):
#            if(신호왔는지 확인, 확인완료)
#                   self._linear_velocity = 0.0
#            if(self._linear_velocity == 0.0)
#                
#                start_time = dt.datetime.now() 장애물 발견시각
#                time.sleep(3.0)
#                end_time = dt.datetime.now() 3초 지났는지 확인용
#                if(3초 지났으면 end_time - start_time >= 3.0)       
#                   self._linear_velocity = 6.0
#                elif(아직 안지났으면 end_time - start_time <= 3.0) 오차 발생 방지
#                   time.sleep(0.01) 0.01 초씩 슬립하고 재확인
#                   end_time = dt.datetime.now() 

        @property
        def linear_velocity(self) -> float:
            return self._linear_velocity

        @property
        def angular_velocity(self) -> float:
            return self._angular_velocity


def main():
    vel = Velocity()
    print('thread info:', vel.thread)
    for i in range(100):
        print('linear = %f, angular = %f' %
              (vel.linear_velocity, vel.angular_velocity))
        time.sleep(0.5)

    vel.thread.join()
    print('thread closed...')
    print('thread info:', vel.thread)


if __name__ == '__main__':
    main()
