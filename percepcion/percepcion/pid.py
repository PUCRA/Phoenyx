class pid():
    def __init__(self, kp, ki, kd, setpoint, max_val = 100000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.max_val = max_val

    def update(self, feedback, dt):
        error = (self.setpoint - feedback)
        self.integral += error*dt
        derivative = (error - self.prev_error)/dt
        self.prev_error = error
        value = self.kp * error + self.ki * self.integral + self.kd * derivative
        if value > self.max_val:
            value = self.max_val
            self.integral = 0
        return value

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0

    def set_max_val(self, max_val):
        self.max_val = max_val

    def reset(self):
        self.integral = 0
        self.prev_error = 0