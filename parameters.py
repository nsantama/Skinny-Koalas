import numpy as np
import cv2

##### Parámetros #####

SERIAL_PORT = "COM5"
PIX_2_M = 1.46 / 457
RADIO_ROBOT = 0.2 / 2
RADIO_RUEDA = 0.09 / 2

# Dato de procesamiento de imágenes: distancia entre CM y centro de uno de los círculos
DATO = 0 # 4,5 cm es el dato real

# Datos del robot respecto al parámetro
ALTO = 4.7
ANCHO = 4.7
DIST_MAYOR_CM = 2.9 # distancia mayor entre el centro de masa del robot y alguno de los márgenes de su estructura

# Proporciones de tamaño respecto al parámetro
RADIO_PELOTA_P = 0.8
LARGO_ENEMIGO_P = 4.7
LARGO_CANCHA_P = 52.2
ANCHO_CANCHA_P = 25.6
LARGO_LIM_ARCO_CANCHA = 3.6 # largo entre el límite de la cancha y el comienzo de área del arco
ANCHO_LIM_ARCO_CANCHA = 8.2 # ancho entre el límite de la cancha y el comienzo de área del arco
MARGEN_D_ARCO = 4.7 # distancia de margen en la que el robot se posicionará para defender el arco
DIST_ADEL_A = 1.7
DIST_LAR_A = 1.7
DIST_LAR_AD_A = 0.7

# Proporciones de tamaño respecto al ancho del robot
ANCHO_ENEMIGO_P = 4.7


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
KP_DIST = 20#0.5
KI_DIST = 0#15
KD_DIST = 0#0.01
KP_ANG = 20
KI_ANG = 0.1
KD_ANG = 0.1

MARGEN_DIST = 0.01
MARGEN_ANG = np.deg2rad(15)
MAX_CONTROL_DIST = 200
MAX_CONTROL_ANG = 200