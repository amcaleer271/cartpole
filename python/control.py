

def bang_bang(state, deadzone, force):
    u = 0.0
    if state[1] > deadzone:
        u = 1 * force
    elif state[1] < deadzone:
        u = -1.0 * force
    else:
        u = 0.0
    return u

class PID:
    def __init__(self, kp, ki, kd):
        self.error = 0.0
        self.error_d = 0.0
        self.error_i = 0.0

        self.kp = kp
        self.ki = ki
        self.kd = kd

    def control(self, state, dt):

        self.error = state[1]
        self.error_d = state[3]
        self.error_i = self.error_i + self.error * dt

        u = self.kp * self.error + self.ki * self.error_i + self.kd * self.error_d

        return u
