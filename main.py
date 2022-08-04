import camera_read as camera
from control_pos import ControlPos
import threading
import time
import serial
import numpy as np


class Brain:
    def __init__(self):
        self.camara = threading.Thread(target=camera.camerarun)
        self.controlpos = ControlPos()
        self.controltr = threading.Thread(target=self.control, daemon=True)
        self.sendinfo = threading.Thread(target=self.send_info, daemon=True)
        self.msg = "0,0;"
        self.emergency = False

    def control(self):
        # while abs(self.error_dist) > self.margen_dist or abs(self.error_ang) > self.margen_ang:
        while (camera.running):
            self.controlpos.error_dist__ = self.controlpos.error_dist_
            self.controlpos.error_dist_ = self.controlpos.error_dist
            self.controlpos.error_dist = np.linalg.norm(camera.robot_center - self.controlpos.posRef)
            self.controlpos.error_ang__ = self.controlpos.error_ang_
            self.controlpos.error_ang_ = self.controlpos.error_ang
            self.controlpos.error_ang = camera.robot_angle - self.controlpos.angRef
            x, y = self.controlpos.get_control()
            self.msg = f"{int(x)},{int(y)};"
        self.msg = "0,0;"
        print("aaaaaaaaaaaa")

    def start(self):
        self.camara.start()
        print("Abriendo camara...")
        while True:
            try:
                print(camera.frame.shape)
                break
            except AttributeError:
                time.sleep(0.5)
        self.sendinfo.start()
        self.controltr.start()

    def send_info(self):
        ser = serial.Serial("COM10", baudrate=38400, timeout=1)
        time.sleep(1)

        while camera.running:
            print(f"Enviando {self.msg}")
            msgEncode = str.encode(self.msg)
            ser.write(msgEncode)
            time.sleep(0.5)
        # Cerramos el puerto serial abierto una vez terminado el codigo
        ser.close()

    def control_ref(self):
        while camera.running:
            self.set_pos_ref(camera.obj)

    def set_pos_ref(self, new_pos):
        # print((np.array(camera.frame.shape)[:2]/2).astype(int))
        # self.controlpos.posRef = (np.array(camera.frame.shape)[:2]/2).astype(int)
        self.controlpos.posRef = new_pos


if __name__ == '__main__':
    cerebro = Brain()
    cerebro.start()
    time.sleep(2)
    cerebro.set_pos_ref(camera.center3)
    print(cerebro.controlpos.posRef)
    print("fin")
