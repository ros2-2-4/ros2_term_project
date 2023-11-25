import cv2
import numpy


class LineTracker:
    def __init__(self):
        self._delta = 0.0
        # self._delta_yellow = 0.0  # Separate delta for yellow line
        self._delta_white = 0.0
        self._white_detected = False

    def process(self, img: numpy.ndarray) -> None:
        """
        calculate the delta from the image
        :param img: Input image
        :return: None
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # yellow color
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 255])
        # white color
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([255, 55, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = img.shape
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
        search_left = int(w / 7)
        search_right = int(6 * w / 7)
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right] = 0
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self._delta = err
            # self._delta_yellow = err
            # END CONTROL
        cv2.imshow("window", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
        search_left = int(w / 3)  # 좌측 1/3 부분을 마스킹
        search_right = int(w / 3 + 5)
        mask2[0:search_top, 0:w] = 0
        mask2[search_bot:h, 0:w] = 0
        mask2[0:h, 0:search_left] = 0  # 좌측 부분 마스킹
        mask2[0:h, search_right:w] = 0
        M2 = cv2.moments(mask2)
        self._white_detected = M2['m00'] > 0
        #정지선은 1m 흰선 두께가 1m 이상인 선만 탐지하게
        cv2.imshow("window", img)
        cv2.imshow("mask", mask2)
        cv2.waitKey(3)

    @property
    def delta(self):
        return self._delta

    @property
    def white_detected(self):
        return self._white_detected

    # A setter for self._delta is not defined.
    # @delta.setter
    # def delta(self, delta):
    #     self._delta = delta


def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('/root/Ros2Projects/oom_ws/src/ros2_term_project/worlds/sample.png')
        tracker.process(img)
        time.sleep(0.1)


if __name__ == "__main__":
    main()