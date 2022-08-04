import time
import numpy as np
import camera_read as camera


class ControlPos:
    def __init__(self):
        self.posRef = np.array((0, 0))
        self.angRef = 0
        self.control_dist = 0
        self.control_and = 0

        self.Kp_dist = 1
        self.Ki_dist = 0
        self.Kd_dist = 0
        self.Kp_ang = 1
        self.Ki_ang = 0
        self.Kd_ang = 0

        self.error_dist = 0
        self.error_dist_ = 0
        self.error_dist__ = 0
        self.error_ang = 0
        self.error_ang_ = 0
        self.error_ang__ = 0

        self.margen_dist = 0.001
        self.margen_ang = 0.1

        self.control_dist_max = 100
        self.control_dist_min = -100
        self.control_and_max = 3
        self.control_and_min = -3

        self.t = 0
        self.t_ = 0

    def get_coef(self, Ts, Kp, Ki, Kd):
        k0 = Kp * (1 + Ts*Ki + Kd/Ts)
        k1 = -Kp * (1 + 2*Kd/Ts)
        k2 = Kp*Kd/Ts
        return (k0, k1, k2)

    def get_control(self):
        self.t = time.time()
        time.sleep(0.01)
        Ts = self.t - self.t_
        self.t_ = self.t
        k0_dist, k1_dist, k2_dist = self.get_coef(Ts, self.Kp_dist, self.Ki_dist, self.Kd_dist)
        k0_ang, k1_ang, k2_ang = self.get_coef(Ts, self.Kp_ang, self.Ki_ang, self.Kd_ang)
        self.control_dist_ = self.control_dist
        self.control_and_ = self.control_and
        self.control_dist = self.control_dist_ + *self.error_dist + k1_dist*self.error_dist_ + k2_dist*self.error_dist__
        self.control_and = self.control_and_ + k0_ang*self.error_ang + k1_ang*self.error_ang_ + k2_ang*self.error_ang__
        self.control_dist = max(self.control_dist_min, self.control_dist)
        self.control_dist = min(self.control_dist, self.control_dist_max)
        self.control_and = max(self.control_and_min, self.control_and)
        self.control_and = min(self.control_and, self.control_and_max)

        vel_R = self.control_dist + self.control_and
        vel_L = self.control_dist - self.control_and
        return vel_R, vel_L

        """    def send_control():
        global control_dist, self.control_and

        msg = f"{vel_L},{vel_R};"
        print(msg)
        msg_encode = str.encode(msg)
        ser.write(msg_encode)
        time.sleep(0.5)"""

    def make_control(self):
        while abs(self.error_dist) > self.margen_dist or abs(self.error_ang) > self.margen_ang:
            self.get_control()
