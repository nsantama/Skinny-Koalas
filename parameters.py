import numpy as np
import cv2

##### Parámetros #####

SERIAL_PORT = "COM8"
PIX_2_M = 1.0
RADIO_ROBOT = 0.08
RADIO_RUEDA = 0.08


# Datos del robot
ALTO = ""
ANCHO = ""
DIST_MAYOR_CM = "" # distancia mayor entre el centro de masa del robot y alguno de los márgenes de su estructura

# Proporciones de tamaño respecto al largo del robot
RADIO_PELOTA_P = ""
LARGO_ENEMIGO_P = ""
LARGO_CANCHA_P = ""
ANCHO_CANCHA_P = ""
LARGO_LIM_ARCO_CANCHA = "" # largo entre el límite de la cancha y el comienzo de área del arco
ANCHO_LIM_ARCO_CANCHA = "" # ancho entre el límite de la cancha y el comienzo de área del arco
MARGEN_D_ARCO = "" # distancia de margen en la que el robot se posicionará para defender el arco

# Proporciones de tamaño respecto al ancho del robot
ANCHO_ENEMIGO_P = ""


# Camara
a = True
N_CAM = 1
LOWER_COLOR_ERROR = np.array([-10, -35, -35])
UPPER_COLOR_ERROR = np.array([10, 35, 35])

TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
TEXT_SCALE = 0.5
TEXT_COLOR = (255, 255, 255)
TEXT_THICK = 2
GAUSSIAN_KSIZE = (51, 51)


# Controlador
KP_DIST = 0.5
KI_DIST = 15
KD_DIST = 0.01
KP_ANG = 0.5
KI_ANG = 15
KD_ANG = 0.01
