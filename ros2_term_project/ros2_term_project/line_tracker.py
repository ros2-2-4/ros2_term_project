import cv2
import numpy


class LineTracker:
    def __init__(self):
        self._delta = 0.0

    def process(self, img: numpy.ndarray) -> None:
        """
        calculate the delta from the image
        :param img:
        :return: None
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # yellow color
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
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
            # END CONTROL
        cv2.imshow("window", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)

    @property
    def delta(self):
        return self._delta

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

