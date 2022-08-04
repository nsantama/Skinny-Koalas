## Estrategia

## Parametros


## Datos
# Coordenadas frente robot
x_front_r = 0
y_front_r = 0

# Coordenadas trasero (centro) robot
x_back_r = 0
y_back_r = 0

# Coordenadas frente enemigo
x_front_e = 0
y_front_e = 0

# Coordenadas trasero (centro) enemigo
x_back_e = 0
y_back_e = 0

# Coordenadas pelota
x_ball = 0
y_ball = 0

# Coordenadas arco
x_arco = 0
y_arco = 0


## Funciones
# Si el objetivo no esta en la visión del robot, rotar sobre su propio eje hasta que lo encuentre
def rotar():
    pass

# Si el objetivo esta en la línea del robot, avanzar derecho
def avanzar():
    pass

# Si el objetivo está en la visión del robot pero no en su línea, rotar y avanzar a la vez
def avanzar_rotar():
    pass

# Si entre el objetivo y el robot hay un obstaculo, cambiar la posición del objetivo para esquivar
def esquivar():
    pass

# Si es que las ruedas se mueven pero no cambia la posición, retroceder y girar
def desatascar():
    pass