class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.isum = 0
        self.limit = limit
        self.e_prev = 0
        

    def calc(self, desired, actual):
        e = desired - actual
        de = e - self.e_prev

        self.isum = self.isum + e

        output = self.kp*e + self.ki*self.isum + self.kd*de

        if(output > self.limit):
            output = self.limit
        if(output < -self.limit):
            output = -self.limit

        return output