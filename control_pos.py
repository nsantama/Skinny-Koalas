import time
import numpy as np
import serial
import camera_read as camera


Kp_dist = 1
Ki_dist = 0
Kd_dist = 0
Kp_ang = 1
Ki_ang = 0
Kd_ang = 0

error_dist = 0
error_dist_ = 0
error_dist__ = 0
error_ang = 0
error_ang_ = 0
error_ang__ = 0

margen_dist = 0.001
margen_ang = 0.1

control_dist_max = 100
control_dist_min = -100
control_ang_max = 3
control_ang_min = -3

t = 0
t_ = 0

ser = serial.Serial("COM6", baudrate=38400, timeout=10)


def get_coef(Ts, Kp, Ki, Kd):
    k0 = Kp * (1 + Ts*Ki + Kd/Ts)
    k1 = -Kp * (1 + 2*Kd/Ts)
    k2 = Kp*Kd/Ts
    return (k0, k1, k2)


def get_control(posRef, angRef):
    global error_dist, error_dist_, error_dist__
    global error_ang, error_ang_, error_ang__
    global control_dist, control_dist_
    global control_ang, control_ang_
    global t, t_

    error_dist__ = error_dist_
    error_dist_ = error_dist
    error_dist = np.linalg.norm(camera.robot_center - posRef)
    error_ang__ = error_ang_
    error_ang_ = error_ang
    error_ang = camera.robot_angle - angRef
    t = time.time()
    Ts = t - t_
    t_ = t
    k0_dist, k1_dist, k2_dist = get_coef(Ts, Kp_dist, Ki_dist, Kd_dist)
    k0_ang, k1_ang, k2_ang = get_coef(Ts, Kp_ang, Ki_ang, Kd_ang)
    control_dist_ = control_dist
    control_ang_ = control_ang
    control_dist = control_dist_ + k0_dist*error_dist + k1_dist*error_dist_ + k2_dist*error_dist__
    control_ang = control_ang_ + k0_ang*error_ang + k1_ang*error_ang_ + k2_ang*error_ang__
    control_dist = max(control_dist_min, control_dist)
    control_dist = min(control_dist, control_dist_max)
    control_ang = max(control_ang_min, control_ang)
    control_ang = min(control_ang, control_ang_max)


def send_control():
    global control_dist, control_ang

    vel_R = control_dist + control_ang
    vel_L = control_dist - control_ang
    msg = f"{vel_L},{vel_R};"
    print(msg)
    msg_encode = str.encode(msg)
    ser.write(msg_encode)
    time.sleep(0.5)


def make_control(posRef, angRef):
    global error_dist, error_ang
    print(1)
    get_control(posRef, angRef)
    print(2)
    send_control()
    while abs(error_dist) > margen_dist or abs(error_ang) > margen_ang:
        get_control(posRef, angRef)
        send_control()


make_control([240,320],0)