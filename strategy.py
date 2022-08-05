from sympy import Point, Line, Polygon
import numpy as np
import parameters as p
import math


##### Datos #####
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
# Coordenadas arco
x_arco = 0
y_arco = 0
# Ángulo calculado en procesamiento de imágenes (radianes)
ang_robot = 0  # del robot c/r marco referencia
ang_enemigo = 0  # angulo del enemigo con referencia
pos_ball = 0  # posición pelota (sale del procesamiento de imágenes)
ang_ball = 0  # de la pelota c/r a robot. Esto viene del proc de img
pos_arco = 0  # Posicion arco (se saca de los parametros)
objetivo = 0  # posicion del objetivo (no necesariamnet es lo que se le evía al robot)
objetivo = pos_ball  # Por default, el objetivo es la pelota


##### Funciones: Hay que retornar la posición y el ERROR del ángulo #####

def strategy_main(robot_pos_F, robot_pos_C, robot_ang,
                  enemy_pos_F, enemy_pos_C, enemy_ang,
                  ball_pos, ball_ang):
    global x_front_r, y_front_r, x_center_r, y_center_r, ang_robot
    global x_front_e, y_front_e, x_center_e, y_center_e, ang_enemigo
    global pos_ball, ang_ball
    x_front_r, y_front_r = robot_pos_F
    x_center_r, y_center_r = robot_pos_C
    ang_robot = robot_ang
    x_front_e, y_front_e = enemy_pos_F
    x_center_e, y_center_e = enemy_pos_C
    ang_enemigo = enemy_ang
    pos_ball = ball_pos
    ang_ball = ball_ang
    # if atascado(): # Si lleva un rato sin moverse (atascado) que retroceda
        # pos_enviar, ang_enviar = desatascar()
    # else: # Si no está atascado
    objetivo = set_objetivo()  # Se define el punto objetivo
    area = zona_objetivo(objetivo)  # Ve en qué ubicación se encuentra el objetivo
    if area == "fuera":
        #print('fuera')
        pos_enviar, ang_enviar = rotar(objetivo)
    elif area == "visión":
        #print('vision')
        pos_enviar, ang_enviar = avanzar_rotar(objetivo)
    elif area == "ataque":
        #print('ataque')
        pos_enviar, ang_enviar = avanzar(objetivo)
    #print("pos_enviar:", pos_enviar, "Ang_enviar:",ang_enviar)
    return pos_enviar, ang_enviar

# Ver en qué área del robot se encuentra el objetivo, para decidir la
def zona_objetivo(objetivo):
    global ang_ball, x_center_r, y_center_r, ang_robot
    if ang_ball > np.pi/12 or ang_ball < -np.pi/12:
        return "fuera"
    else:
        c_adelante = y_center_r + p.DIST_ADEL_A * p.DATO*2
        c_lado_izq = x_center_r - p.DIST_LAR_A * p.DATO
        c_lado_der = x_center_r + p.DIST_LAR_A * p.DATO
        c_maximo = y_center_r + p.LARGO_CANCHA_P * p.DATO
        e_c = (x_center_r, y_center_r) # Centro de masa robot
        i1 = Point(rotate(e_c, (c_lado_izq, c_adelante), ang_robot))
        s1 = Point(rotate(e_c, (c_lado_izq, c_maximo), ang_robot))
        i2 = Point(rotate(e_c, (c_lado_der, c_adelante), ang_robot))
        s2 = Point(rotate(e_c, (c_lado_der, c_maximo), ang_robot))
        p1, p2, p3, p4 = s1, s2, i1, i2
        area_ataque = Polygon(p1, p2, p3, p4)
        if type(area_ataque) == Polygon:
            if area_ataque.contains(objetivo):
                return "ataque"
            else:
                return "visión"
        else:
            return "visión"

# Rotar un punto en base a un origen y un álgulo
def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

# Si el robot debe rotar, se define el ángulo en grados y si será horario y antihorario
def corregir_angulo(ang):
    if ang > np.pi:
        ang = ang - 2*np.pi
    elif ang < -np.pi:
        ang = 2*np.pi + ang
    return ang

# Define cual es el punto objetivo dependiendo de distintas condiciones
def set_objetivo():
    global pos_ball
    if not en_dominio(): # Si no esta en dominio, que su objetivo sea la pelota
        objetivo = pos_ball
        if bloqueado(objetivo):
            objetivo, _ = esquivar(objetivo)
    elif en_dominio(): # Si está en dominio, entonces que vaya al arco
        objetivo = pos_arco
        if bloqueado(objetivo):
            objetivo, _ = esquivar(objetivo)
    return objetivo

# Define el error del ángulo entre el robot y el objetivo
def set_angulo(objetivo):
    global pos_ball, ang_ball, x_center_r, y_center_r, x_front_r, y_front_r
    if objetivo.all() == pos_ball.all():
        error_ang = ang_ball
    else:
        p_o = Point(objetivo) # Punto objetivo
        p_c = Point(x_center_r, y_center_r) # Punto centro robot
        p_f = Point(x_front_r, y_front_r) # Punto frente robot
        l_r = Line(p_c, p_f) # Linea de la trayectoria del robot
        l_o = Line(p_c, p_o)
        error_ang = l_r.angle_between(l_o)
    return error_ang

# Si el objetivo no esta en la visión del robot, rotar sobre su propio eje hasta que lo encuentre
def rotar(objetivo):
    global x_center_r, y_center_r
    punto_objetivo = (x_center_r, y_center_r)
    error_ang = set_angulo(objetivo)
    return punto_objetivo, error_ang

# Si el objetivo esta en la línea del robot, avanzar derecho
def avanzar(objetivo):
    punto_objetivo = objetivo
    error_ang = 0  # Se define que esta alineado (aunque no sea perfecto)
    return punto_objetivo, error_ang

# Si el objetivo está en la visión del robot pero no en su línea, rotar y avanzar a la vez
def avanzar_rotar(objetivo):
    punto_objetivo = objetivo
    error_ang = set_angulo(objetivo)
    return punto_objetivo, error_ang

# Si entre el objetivo y el robot hay un obstaculo, cambiar la posición del objetivo para esquivar
def esquivar(objetivo):
    global ang_ball, x_center_r, y_center_r
    # Se usa dentro de set_objetivo
    # Setea como objetivo un punto viable que acerque al objetivo y que ya no tenga un objeto entre medio
    # rota y define un nuevo punto hasta que bloqueado sea False
    ang_ball += 0.01
    nuevo_objetivo = rotate((x_center_r, y_center_r), objetivo, ang_ball)
    if bloqueado(nuevo_objetivo) is True:
        return esquivar(nuevo_objetivo)
    else:
        return nuevo_objetivo, ang_ball

# Si es que las ruedas se mueven pero no cambia la posición, retroceder y girar
def desatascar():
    global x_front_r, y_front_r, x_center_r, y_center_r
    # Se usa dentro de set_objetivo
    # Define una pos objetivo atrás del robot
    # El angulo es 0 (esta alineado con su objetivo, aunque esté de espalda, no queremos que rote)
    # Avanza a ese objetivo
    nuevo_x = -3 * (x_front_r - x_center_r)
    nuevo_y = -3 * (y_front_r - y_center_r)
    return (nuevo_x, nuevo_y) , 0

# Devuelve un bool de si está bloqueado hacia su objetivo o no
def bloqueado(objetivo):
    global x_front_r, y_front_r, x_center_e, y_center_e, ang_enemigo
    r = Point(x_front_r, y_front_r) # Punto frente robot
    o = Point(objetivo) # Punto del objetivo
    trayectoria = Line(r, o)
    e_c = (x_center_e, y_center_e) # Centro de masa enemigo
    p_1x_ = x_center_e - (p.ANCHO * p.DATO)/2
    p_2x_ = x_center_e + (p.ANCHO * p.DATO)/2
    p_1y_ = y_center_e + (p.ALTO * p.DATO)/2
    p_2y_ = y_center_e - (p.ALTO * p.DATO)/2
    s1 = Point(rotate(e_c, (p_1x_, p_1y_), ang_enemigo))
    s2 = Point(rotate(e_c, (p_2x_, p_1y_), ang_enemigo))
    i1 = Point(rotate(e_c, (p_1x_, p_2y_), ang_enemigo))
    i2 = Point(rotate(e_c, (p_2x_, p_2y_), ang_enemigo))
    enemigo = Polygon(s1, s2, i1, i2)
    lista_intersect = trayectoria.intersection(enemigo)
    if len(lista_intersect) == 0:
        return False
    else:
        return True

# Si la pelota esta en area, devuelve true
def en_dominio():
    global pos_ball, x_center_r, y_center_r, ang_robot
    pelota = Point(*pos_ball)
    c_adelante = y_center_r + p.DIST_ADEL_A * p.DATO
    c_lado_izq = x_center_r - p.DIST_LAR_A * p.DATO
    c_lado_der = x_center_r + p.DIST_LAR_A * p.DATO
    c_maximo = y_center_r + p.DIST_LAR_AD_A * p.DATO
    e_c = (x_center_r, y_center_r) # Centro de masa robot
    i1 = Point(rotate(e_c, (c_lado_izq, c_adelante), ang_robot))
    s1 = Point(rotate(e_c, (c_lado_izq, c_maximo), ang_robot))
    i2 = Point(rotate(e_c, (c_lado_der, c_adelante), ang_robot))
    s2 = Point(rotate(e_c, (c_lado_der, c_maximo), ang_robot))
    p1, p2, p3, p4 = [(s1), (s2), (i1), (i2)]
    area_dominio = Polygon(p1, p2, p3, p4)
    lista_puntos = pelota.intersection(area_dominio)
    if len(lista_puntos) == 0:
        return False
    else:
        return True

def atascado():
    pass
    # Si las ruedas se mueven y no avanza, retorna true