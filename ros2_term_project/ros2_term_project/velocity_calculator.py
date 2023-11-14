from enum import Enum


class VelocityCalculator:
    ERROR = 1e-3

    def __init__(self, rate: float, duration: float, first_value: float, target_value: float):
        self.rate = rate
        self.duration = duration
        self.first_value = first_value
        self.target_value = target_value
        self.current_value = first_value
        self.step_value = (self.target_value - self.first_value) / (self.rate * self.duration)
        self._state = VelocityCalculator.State.INIT

    def update_parameter(self, current: float, target: float):
        self.current_value = current
        self.target_value = target

    def next_value(self) -> float:
        if abs(self.target_value - self.current_value) < VelocityCalculator.ERROR:
            self.set_state(VelocityCalculator.State.REACHED)
            return self.target_value
        self.current_value = self.current_value + self.step_value
        self._state = VelocityCalculator.State.CHANGED
        return self.current_value

    @property
    def state(self):
        return self._state

    def set_state(self, new_state: 'VelocityCalculator.State'):
        self._state = new_state

    class State(Enum):
        INIT = 0
        CHANGED = 1
        REACHED = 2


def main():
    rate = 5
    duration = 2
    first_value = 0.
    target_value = 5.

    print('rate = %f, duration = %f' % (rate, duration))
    print('first_value = %f, target_value = %f' % (first_value, target_value))

    calculator = VelocityCalculator(rate, duration, first_value, target_value)
    import time
    import datetime as dt

    start_time = dt.datetime.now()
    for i in range(rate * duration):
        time.sleep(1.0/rate)
        print('next value = %f' % calculator.next_value())

    end_time = dt.datetime.now()
    print('elapsed time = %s' % str((end_time - start_time).total_seconds()))


if __name__ == '__main__':
    main()

