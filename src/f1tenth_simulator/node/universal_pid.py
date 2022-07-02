#reinvented by Kristoff - zhikangliu
import time

class universal_pid:
    def __init__(self, _kP, _kI, _kD, _windup_guard = 20.0):
        self.kP = _kP
        self.kI = _kI
        self.kD = _kD

        self.current_time = time.time()
        self.previous_time = self.current_time
        self.reset()

    def reset(self):
        self.proportion = 0
        self.integral = 0
        self.differential = 0

        self.previous_error = 0
        self.current_error = 0
        self.output = 0
    
    def update(self, error_input):
        # work out the change of time in the discrete status
        self.current_time = time.time()
        delta_time = self.current_time - self.previous_time

        # work out the change of error in the discrete status
        self.current_error = error_input
        delta_error = self.current_error - self.previous_error

        self.proportion = self.current_error

        self.integral += self.current_error * delta_time

        if self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        elif self.integral > self.windup_guard:
            self.integral = self.windup_guard

        self.differential = delta_error / delta_time

        self.previous_time = self.current_time
        self.previous_error = self.current_error

        self.output = self.kP * self.proportion  + self.kI * self.integral + self.kD * self.differential
        return self.output

    def set_kP(self, _kP):
        self.kP = _kP

    def set_kI(self, _kI):
        self.kI = _kI

    def set_kD(self, _kD):
        self.kD = _kD

    def set_windup_guard(self, _windup_guard = 20.0):
        self.windup_guard = _windup_guard

        