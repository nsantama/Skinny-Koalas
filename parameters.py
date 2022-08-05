import numpy as np
import cv2

##### Parámetros #####

SERIAL_PORT = "COM8"

# Datos del robot
ALTO = ""
ANCHO = ""
DIST_MAYOR_CM = "" # distancia mayor entre el centro de masa del robot y alguno de los márgenes de su estructura
RADIO = 0.08

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
KP_DIST = 1
KI_DIST = 0
KD_DIST = 0
KP_ANG = 5
KI_ANG = 0
KD_ANG = 0
