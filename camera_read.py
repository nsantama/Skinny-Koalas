from control_pos import ControlPos
import cv2
import threading
import time
import serial
import numpy as np
import os
import parameters as p

running = True
auto = True

robot_center = np.array((0, 0))
robot_angle = 0
color1_hsv = np.array([0, 0, 0])
color2_hsv = np.array([0, 0, 0])
color3_hsv = np.array([0, 0, 0])
color4_hsv = np.array([0, 0, 0])
color5_hsv = np.array([0, 0, 0])


def write_colors():
    global color1_hsv, color2_hsv, color3_hsv, color4_hsv, color5_hsv
    with open(os.path.join("colors", 'color1.npy'), 'wb') as color:
        np.save(color, color1_hsv)
    with open(os.path.join("colors", 'color2.npy'), 'wb') as color:
        np.save(color, color2_hsv)
    with open(os.path.join("colors", 'color3.npy'), 'wb') as color:
        np.save(color, color3_hsv)
    with open(os.path.join("colors", 'color4.npy'), 'wb') as color:
        np.save(color, color4_hsv)
    with open(os.path.join("colors", 'color5.npy'), 'wb') as color:
        np.save(color, color5_hsv)

def read_colors():
    global color1_hsv, color2_hsv, color3_hsv, color4_hsv, color5_hsv
    with open(os.path.join("colors", 'color1.npy'), 'rb') as color:
        color1_hsv = np.load(color)
    with open(os.path.join("colors", 'color2.npy'), 'rb') as color:
        color2_hsv = np.load(color)
    with open(os.path.join("colors", 'color3.npy'), 'rb') as color:
        color3_hsv = np.load(color)
    with open(os.path.join("colors", 'color4.npy'), 'rb') as color:
        color4_hsv = np.load(color)
    with open(os.path.join("colors", 'color5.npy'), 'rb') as color:
        color5_hsv = np.load(color)

def _mouseEvent(event, x, y, flags, param):
    global nClick, frame, obj, color1_hsv, color2_hsv, color3_hsv, color4_hsv, color5_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if nClick == 1:
            color1_hsv = hsv_frame[y, x]
            print(f"Color 1: {color1_hsv}  |  Posicion: {[x, y]}")
            nClick += 1
        elif nClick == 2:
            color2_hsv = hsv_frame[y, x]
            print(f"Color 2: {color2_hsv}  |  Posicion: {[x, y]}")
            nClick += 1
        elif nClick == 3:
            color3_hsv = hsv_frame[y, x]
            print(f"Color 3: {color3_hsv}  |  Posicion: {[x, y]}")
            obj = (x, y)
            nClick += 1
        elif nClick == 4:
            color4_hsv = hsv_frame[y, x]
            print(f"Color 4: {color4_hsv}  |  Posicion: {[x, y]}")
            obj = (x, y)
            nClick += 1
        elif nClick == 5:
            color5_hsv = hsv_frame[y, x]
            print(f"Color 5: {color5_hsv}  |  Posicion: {[x, y]}")
            obj = (x, y)
            nClick += 1
            write_colors()
        else:
            color1_hsv = np.array([0, 0, 0])
            color2_hsv = np.array([0, 0, 0])
            color3_hsv = np.array([0, 0, 0])
            color4_hsv = np.array([0, 0, 0])
            color5_hsv = np.array([0, 0, 0])
            print("Colores reiniciados")
            nClick = 1

def get_mask(hsv, color):
    lower_color = color + p.LOWER_COLOR_ERROR
    upper_color = color + p.UPPER_COLOR_ERROR
    color_mask = cv2.inRange(hsv, lower_color, upper_color)
    color_blur = cv2.GaussianBlur(color_mask, p.GAUSSIAN_KSIZE, 0)
    ret_color, color_mask_OTSU = cv2.threshold(color_blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return color_mask_OTSU

def get_mass_center(mask):
    mom = cv2.moments(mask)
    try:
        x = int(mom["m10"] / mom["m00"])
        y = int(mom["m01"] / mom["m00"])
        return np.array([x, y])
    except ZeroDivisionError as error:
        return None

def draw_centers(img):
    global center1, center2, center3, center4, center5
    color_mc1 = (255, 0, 0)
    color_mc2 = (0, 0, 255)
    color_mc3 = (0, 255, 0)
    color_mc4 = (0, 255, 255)
    color_mc5 = (255, 255, 0)
    if center1 is not None:
        cv2.circle(img, center1, 10, color_mc1, -1)
    if center2 is not None:
        cv2.circle(img, center2, 10, color_mc2, -1)
    if center3 is not None:
        cv2.circle(img, center3, 10, color_mc3, -1)
    if center3 is not None:
        cv2.circle(img, center4, 10, color_mc4, -1)
    if center3 is not None:
        cv2.circle(img, center5, 10, color_mc5, -1)


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
        while True:
            self.set_pos_ref(obj)

    def set_pos_ref(self, new_pos):
        self.controlpos.posRef = new_pos

    def set_ang(self, new_ang):
        self.controlpos.angAct = new_ang


if __name__ == '__main__':
    cap = cv2.VideoCapture(p.N_CAM)
    nClick = 1
    cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow('frame', 30, 100)
    cv2.namedWindow('res', cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow('res', 700, 100)
    cv2.setMouseCallback('frame', _mouseEvent)
    read_colors()
    cerebro = Brain()
    cerebro.start()

    while True:
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        Color1Mask = get_mask(hsv, color1_hsv)
        Color2Mask = get_mask(hsv, color2_hsv)
        Color3Mask = get_mask(hsv, color3_hsv)
        Color4Mask = get_mask(hsv, color4_hsv)
        Color5Mask = get_mask(hsv, color5_hsv)
        Color1Res = cv2.bitwise_and(frame, frame, mask=Color1Mask)
        Color2Res = cv2.bitwise_and(frame, frame, mask=Color2Mask)
        Color3Res = cv2.bitwise_and(frame, frame, mask=Color3Mask)
        Color4Res = cv2.bitwise_and(frame, frame, mask=Color4Mask)
        Color5Res = cv2.bitwise_and(frame, frame, mask=Color5Mask)
        res = Color1Res + Color2Res + Color3Res + Color4Res + Color5Res
        center1 = get_mass_center(Color1Mask)  # Front
        center2 = get_mass_center(Color2Mask)  # Back
        center3 = get_mass_center(Color3Mask)  # Ball
        center4 = get_mass_center(Color4Mask)  # Enemy Front
        center5 = get_mass_center(Color5Mask)  # Enemy Back
        draw_centers(res)

        if center1 is not None and center2 is not None:
            cv2.line(res, center1, center2, (255, 255, 255), 2)
            robot_center = np.array((0.5 * (center1 + center2)).astype(int))
            robot_delta = center1 - center2
            robot_angle = np.arctan2(robot_delta[1], robot_delta[0])
            cv2.circle(res, robot_center, 10, (255, 255, 255), 1)
            cv2.putText(res,  f"Robot: [{robot_center[0]}, {robot_center[1]}, {round(np.rad2deg(robot_angle), 1)}]",
                        (0, 25), p.TEXT_FONT, p.TEXT_SCALE, p.TEXT_COLOR, p.TEXT_THICK)

            if center3 is not None:  # Pelota
                cv2.line(res, robot_center, center3, (255, 255, 255), 2)
                ball_dist = np.linalg.norm(center3 - robot_center)
                ball_delta = center3 - robot_center
                ball_angle = np.arctan2(ball_delta[1], ball_delta[0]) - robot_angle
                if ball_angle > np.pi:
                    ball_angle = ball_angle - 2*np.pi
                elif ball_angle < -np.pi:
                    ball_angle = ball_angle + 2*np.pi
                cv2.putText(res, f"Ball: [{round(ball_dist)}, {round(np.rad2deg(ball_angle), 1)}]",
                            (0, 75), p.TEXT_FONT, p.TEXT_SCALE, p.TEXT_COLOR, p.TEXT_THICK)

            if center4 is not None and center5 is not None:  # Enemigo
                cv2.line(res, center4, center5, (255, 255, 255), 2)
                enemy_center = np.array((0.5 * (center4 + center5)).astype(int))
                enemy_delta = center4 - center5
                enemy_angle = np.arctan2(enemy_delta[1], enemy_delta[0])
                to_enemy_delta = enemy_center - robot_center
                to_enemy_angle = np.arctan2(to_enemy_delta[1], to_enemy_delta[0]) - robot_angle
                if to_enemy_angle > np.pi:
                    to_enemy_angle = to_enemy_angle - 2*np.pi
                elif to_enemy_angle < -np.pi:
                    to_enemy_angle = to_enemy_angle + 2*np.pi
                cv2.circle(res, enemy_center, 10, (255, 255, 255), 1)
                cv2.putText(res,  f"Enemy: [{enemy_center[0]}, {enemy_center[1]}, {round(np.rad2deg(enemy_angle), 1)}]",
                            (0, 50), p.TEXT_FONT, p.TEXT_SCALE, p.TEXT_COLOR, p.TEXT_THICK)

        # Actualizar a donde ir
        new_pos = center3
        new_angle = ball_angle
        cerebro.set_ang(new_angle)
        cerebro.set_pos_ref(new_pos)

        cv2.imshow('frame', frame)
        cv2.imshow('res', res)
        if auto:
            cerebro.msg = f"{cerebro.veld},{cerebro.veli};"
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cerebro.msg = f"0,0;"
    time.sleep(0.1)
    cap.release()
    cv2.destroyAllWindows()
