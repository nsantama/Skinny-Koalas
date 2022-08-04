import cv2
import numpy as np


nCam = 0
LowerColorError = np.array([-10, -35, -35])
UpperColorError = np.array([10, 35, 35])
gaussian_ksize = (51, 51)
text_font = cv2.FONT_HERSHEY_SIMPLEX
text_scale = 0.5
text_color = (255, 255, 255)
text_thick = 2


def write_colors():
    global color1_hsv, color2_hsv, color3_hsv
    with open('color1.npy', 'wb') as color:
        np.save(color, color1_hsv)
    with open('color2.npy', 'wb') as color:
        np.save(color, color2_hsv)
    with open('color3.npy', 'wb') as color:
        np.save(color, color3_hsv)

def read_colors():
    global color1_hsv, color2_hsv, color3_hsv
    with open('color1.npy', 'rb') as color:
        color1_hsv = np.load(color)
    with open('color2.npy', 'rb') as color:
        color2_hsv = np.load(color)
    with open('color3.npy', 'rb') as color:
        color3_hsv = np.load(color)

def _mouseEvent(event, x, y, flags, param):
    global nClick, color1_hsv, color2_hsv, color3_hsv

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
            nClick += 1
            write_colors()
        else:
            color1_hsv = np.array([0, 0, 0])
            color2_hsv = np.array([0, 0, 0])
            color3_hsv = np.array([0, 0, 0])
            print("Colores reiniciados")
            nClick = 1


def get_mask(hsv, color, lower_error, uppper_error):
    global gaussian_ksize
    lower_color = color + lower_error
    upper_color = color + uppper_error
    color_mask = cv2.inRange(hsv, lower_color, upper_color)
    color_blur = cv2.GaussianBlur(color_mask, gaussian_ksize, 0)
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


def draw_centers(img, mass_center1, mass_center2, mass_center3):
    color_mc1 = (255, 0, 0)
    color_mc2 = (0, 0, 255)
    color_mc3 = (0, 255, 0)
    if mass_center1 is not None:
        cv2.circle(img, mass_center1, 10, color_mc1, -1)
    if mass_center2 is not None:
        cv2.circle(img, mass_center2, 10, color_mc2, -1)
    if mass_center3 is not None:
        cv2.circle(img, mass_center3, 10, color_mc3, -1)


cap = cv2.VideoCapture(nCam)
nClick = 1
cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
cv2.moveWindow('frame', 30, 100)
cv2.namedWindow('res', cv2.WINDOW_AUTOSIZE)
cv2.moveWindow('res', 700, 100)
cv2.setMouseCallback('frame', _mouseEvent)
read_colors()
while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Color1Mask = get_mask(hsv, color1_hsv, LowerColorError, UpperColorError)
    Color2Mask = get_mask(hsv, color2_hsv, LowerColorError, UpperColorError)
    Color3Mask = get_mask(hsv, color3_hsv, LowerColorError, UpperColorError)
    Color1Res = cv2.bitwise_and(frame, frame, mask=Color1Mask)
    Color2Res = cv2.bitwise_and(frame, frame, mask=Color2Mask)
    Color3Res = cv2.bitwise_and(frame, frame, mask=Color3Mask)
    res = Color1Res + Color2Res + Color3Res
    center1 = get_mass_center(Color1Mask)  # Front
    center2 = get_mass_center(Color2Mask)  # Back
    center3 = get_mass_center(Color3Mask)  # Ball
    draw_centers(res, center1, center2, center3)

    if center1 is not None and center2 is not None:
        cv2.line(res, center1, center2, (255, 255, 255), 2)
        robot_center = (0.5 * (center1 + center2)).astype(int)
        robot_delta = center1 - center2
        robot_angle = np.arctan2(robot_delta[1], robot_delta[0])
        cv2.circle(res, robot_center, 10, (255, 255, 255), 1)
        cv2.putText(res,  f"Robot: [{robot_center[0]}, {robot_center[1]}, {round(np.rad2deg(robot_angle), 1)}]",
                    (0, 25), text_font, text_scale, text_color, text_thick)

        if center3 is not None:
            cv2.line(res, robot_center, center3, (255, 255, 255), 2)
            ball_dist = np.linalg.norm(center3 - robot_center)
            ball_delta = center3 - robot_center
            ball_angle = np.arctan2(ball_delta[1], ball_delta[0]) - robot_angle
            cv2.putText(res, f"Ball: [{round(ball_dist)}, {round(np.rad2deg(ball_angle), 1)}]",
                        (0, 75), text_font, text_scale, text_color, text_thick)

    cv2.imshow('frame', frame)
    cv2.imshow('res', res)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
