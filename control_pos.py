import time
import numpy as np
import parameters as p


class ControlPos:
    def __init__(self):
        self.posRef = np.array((0, 0))
        self.angRef = 0
        self.control_dist = 0
        self.control_ang = 0

        self.Kp_dist = p.KP_DIST
        self.Ki_dist = p.KI_DIST
        self.Kd_dist = p.KD_DIST
        self.Kp_ang = p.KP_ANG
        self.Ki_ang = p.KI_ANG
        self.Kd_ang = p.KD_ANG

        self.error_dist = 0
        self.error_dist_ = 0
        self.error_dist__ = 0
        self.error_ang = 0
        self.error_ang_ = 0
        self.error_ang__ = 0

        self.margen_dist = 0.001
        self.margen_ang = 0.1

        self.control_dist_max = 10
        self.control_dist_min = -10
        self.control_ang_max = 3
        self.control_ang_min = -3

        self.t = 0
        self.t_ = 0

    def get_coef(self, Ts, Kp, Ki, Kd):
        k0 = Kp * (1 + Ts*Ki + Kd/Ts)
        k1 = -Kp * (1 + 2*Kd/Ts)
        k2 = Kp*Kd/Ts
        return (k0, k1, k2)

    def get_control(self, Ts):
        k0_dist, k1_dist, k2_dist = self.get_coef(Ts, self.Kp_dist, self.Ki_dist, self.Kd_dist)
        k0_ang, k1_ang, k2_ang = self.get_coef(Ts, self.Kp_ang, self.Ki_ang, self.Kd_ang)
        self.control_dist_ = self.control_dist
        self.control_ang_ = self.control_ang
        self.control_dist = self.control_dist_ + k0_dist*self.error_dist + k1_dist*self.error_dist_ + k2_dist*self.error_dist__
        self.control_ang = self.control_ang_ + k0_ang*self.error_ang + k1_ang*self.error_ang_ + k2_ang*self.error_ang__
        #self.control_dist = max(self.control_dist_min, self.control_dist)
        #self.control_dist = min(self.control_dist, self.control_dist_max)
        #self.control_ang = max(self.control_ang_min, self.control_ang)
        #self.control_ang = min(self.control_ang, self.control_ang_max)

        vel_R = (self.control_dist + self.control_ang) / p.RADIO
        vel_L = (self.control_dist - self.control_ang) / p.RADIO
        return -vel_R, -vel_L

        """    def send_control():
        global control_dist, self.control_ang

        msg = f"{vel_L},{vel_R};"
        print(msg)
        msg_encode = str.encode(msg)
        ser.write(msg_encode)
        time.sleep(0.5)"""

    def make_control(self):
        while abs(self.error_dist) > self.margen_dist or abs(self.error_ang) > self.margen_ang:
            self.get_control()
