## Estrategia

## Librerías
from sympy import Point, Line, 

## Parámetros 
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

## Datos
# Coordenadas frente robot
x_front_r = 0
y_front_r = 0

# Coordenadas trasero (centro) robot
x_center_r = 0
y_center_r = 0

# Coordenadas frente enemigo
x_front_e = 0
y_front_e = 0

# Coordenadas trasero (centro) enemigo
x_center_e = 0
y_center_e = 0

# Coordenadas pelota
x_ball = 0
y_ball = 0

# Coordenadas arco
x_arco = 0
y_arco = 0

# Ángulo calculado en procesamiento de imágenes
ang = 0

## Funciones: Hay que retornar la posición y el ángulo
# Si el robot debe rotar, se define el ángulo en grados y si será horario y antihorario
def angulo():
    if ang > 180 and ang < 270:
        ang = -1 * (360 - ang)
    if ang == 270:
        ang = -90
    if ang > 270 and ang < 360:
        ang = -1 * (360 - ang)
    if ang == 360:
        ang = 0
    return ang
        
# Define cual es el punto objetivo dependiendo de distintas condiciones
def set_objetivo():
    pass

# Si el objetivo no esta en la visión del robot, rotar sobre su propio eje hasta que lo encuentre
def rotar():
    objetivo = (x_center_r, y_center_r)
    angulo = angulo()
    return objetivo, angulo

# Si el objetivo esta en la línea del robot, avanzar derecho
def avanzar():
    objetivo = set_objetivo()
    angulo = 0 # Se define que esta alineado (aunque no sea perfecto)
    return objetivo, angulo

# Si el objetivo está en la visión del robot pero no en su línea, rotar y avanzar a la vez
def avanzar_rotar():
    objetivo = set_objetivo()
    angulo = angulo()
    return objetivo, angulo

# Si entre el objetivo y el robot hay un obstaculo, cambiar la posición del objetivo para esquivar
def esquivar():
    # Se usa dentro de set_objetivo
    # Setea como objetivo un punto viable que acerque al objetivo y que ya no tenga un objeto entre medio
    # o quizas engañarlo (?) y hacer que avance y rote para hacer una curva
    pass

# Si es que las ruedas se mueven pero no cambia la posición, retroceder y girar
def desatascar():
    # Se usa dentro de set_objetivo
    # Define una pos objetivo atrás del robot
    # El angulo es 0 (esta alineado con su objetivo, aunque esté de espalda, no queremos que rote)
    # Avanza a ese objetivo
    pass 

# Devuelve un bool de si está bloqueado hacia su objetivo o no
def bloqueado():
    r = Point(x_front_r, y_front_e) # Punto frente robot
    o = Point(set_objetivo()) # Punto del objetivo
    trayectoria = Line(r, o)
    enemigo = 
    pass