import threading
import time
import numpy as np
import serial
from control_pos import ControlPos
import parameters as p


class Brain:
    def __init__(self):
        self.controlpos = ControlPos()
        self.controltr = threading.Thread(target=self.control, daemon=True)
        self.sendinfo = threading.Thread(target=self.send_info, daemon=True)
        self.msg = "0,0;"
        self.emergency = False
        self.veld = 0
        self.veli = 0

    def control(self):
        global robot_center
        controling = True
        while controling:
            self.controlpos.t = time.time()
            time.sleep(0.01)
            Ts = self.controlpos.t - self.controlpos.t_
            self.controlpos.t_ = self.controlpos.t
            self.controlpos.error_dist__ = self.controlpos.error_dist_
            self.controlpos.error_dist_ = self.controlpos.error_dist
            self.controlpos.error_dist = np.linalg.norm(robot_center - self.controlpos.posRef) * p.PIX_2_M
            self.controlpos.error_ang__ = self.controlpos.error_ang_
            self.controlpos.error_ang_ = self.controlpos.error_ang
            self.controlpos.error_ang = self.controlpos.angAct - self.controlpos.angRef
            self.veld, self.veli = self.controlpos.get_control(Ts)

    def start(self):
        print("paso1")
        self.sendinfo.start()
        print("paso2")
        self.controltr.start()

    def send_info(self):
        ser = serial.Serial(p.SERIAL_PORT, baudrate=38400, timeout=100)
        time.sleep(1)

        while True:
            print(f"Enviando {self.msg}")
            #print(self.controlpos.error_dist)
            msgEncode = str.encode(self.msg)
            ser.write(msgEncode)
            #time.sleep(0.5)
        ser.close()

    def control_ref(self):
        global obj
        while True:
            self.set_pos_ref(obj)

    def set_pos_ref(self, new_pos):
        self.controlpos.posRef = new_pos

    def set_ang(self, new_ang):
        self.controlpos.angAct = new_ang
