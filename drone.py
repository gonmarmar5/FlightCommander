'''
avoid.py

Sample client for the Pioneer P3DX mobile robot that implements a
kind of heuristic, rule-based controller for collision avoidance.

Copyright (C) 2023 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import robotica_drone
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np

def where_to_go(current_dir, readings):
    right = False
    left = False

    # Hemos visto que si le queremos dar la misma prioridad a la derecha que a la izquierda, puede pasar que si gira primero a la izquierda, después vaya a girar a la derecha, después a la izquierda... Así que para arreglar eso, si empieza el giro hacia la izquierda no puede ir a la derecha hasta que haya acabado el giro (hasta que haya ido marcha atras o recto hacia delante)
    if current_dir == 1:
        right = True
        current_dir = 1
        return right, left, current_dir
    elif current_dir == -1:
        left = True
        current_dir = -1
        return right, left, current_dir

    going_to_crash = 0
    going_to_crash_2 = []
    going_to_crash_3 = []
    going_to_crash_4 = []
    going_to_crash_5 = []

    # Primero recolectamos todas las medidas de los principales sensores y de los sensores que detectan el objeto
    threshold = [0.3, 0.4, 0, 0.4, 0.3]
    for i in range(1, 6):
        if i != 3:
            going_to_crash_4.append(readings[i])
            going_to_crash_5.append(i)
            if readings[i] < threshold[i - 2]:
                going_to_crash_2.append(readings[i])
                going_to_crash_3.append(i)
                going_to_crash += 1
    
    try:
        top_going_to_crush = going_to_crash_3[np.argmin(going_to_crash_2, -1)] # mirarmos cuál está más cerca de chocar
        top_save_rute = going_to_crash_5[np.argmax(going_to_crash_4, -1)] # miramos cuál tiene la mejor ruta sin chocar

        if top_going_to_crush < 3 and top_save_rute > 3 and (current_dir == 1 or current_dir == 0): # si la mejor ruta es a la derecha y la que está mas cerca de chocar es a la izquierda, gira a la derecha
            right = True
            current_dir = 1
            return right, left, current_dir
        elif top_going_to_crush > 3 and top_save_rute < 3 and (current_dir == -1 or current_dir == 0): # si la mejor ruta es a la izquierda y la que está mas cerca de chocar es a la derecha, gira a la izquierda
            left = True
            current_dir = -1
            return right, left, current_dir
        else: # si no, mira la media de los dos sensores y quedate con el que tenga la media más grande, ya que de media, en esa dirección los objetos detectados están más lejos
            if (readings[5] + readings[4]) / 2 > (readings[2] + readings[1]) / 2 and (current_dir == 1 or current_dir == 0):
                right = True
                current_dir = 1
                return right, left, current_dir
            elif (readings[5] + readings[4]) / 2 <= (readings[2] + readings[1]) / 2 and (current_dir == -1 or current_dir == 0):
                left = True
                current_dir = -1
                return right, left, current_dir
    except: # solo hay un error si el único sensor que detecta el objeto es el 3 (el de seguir recto). En ese caso calcula la media y ves para el que tenga los objetos más lejos
        if (readings[5] + readings[4]) / 2 > (readings[2] + readings[1]) / 2 and (current_dir == 1 or current_dir == 0):
            right = True
            current_dir = 1
            return right, left, current_dir
        elif (readings[5] + readings[4]) / 2 <= (readings[2] + readings[1]) / 2 and (current_dir == -1 or current_dir == 0):
            left = True
            current_dir = -1
            return right, left, current_dir

    return current_dir, right, left


def avoid(readings, current_dir):
    print(readings)
    print(current_dir)
    
    #readings[0] # lateral izquierda delante rueda
    #readings[1] # delante izquierda
    #readings[2] # un poco izquierda
    #readings[3] # recto
    #readings[4] # un poco derecha
    #readings[5] # delante derecha
    #readings[6] # delante muy derecha
    #readings[7] # lateral dercha delante rueda
    #readings[8] # lateral derecha detras rueda
    #readings[9] # derecha marcha atras
    #readings[10] # un poco derecha marcha atras
    #readings[11] # marcha atras
    #readings[12] # un poco izquierda marcha atras
    #readings[13] # izquierda marcha atras
    #readings[14] # bastante izquierda marcha atras
    #readings[15] # lateral izquierda detras rueda

    lspeed, rspeed = +1.5, +1.5
    reading = -1

    # marcha atras
    if readings[3] < 0.1 or readings[4] < 0.1 or readings[2] < 0.1 or readings[5] < 0.1 or readings[1] < 0.1 or (readings[3] < 0.2 and readings[4] < 0.2 and readings[2] < 0.2 and readings[1] < 0.2 and readings[0] < 0.2 and readings[5] < 0.2 and readings[7] < 0.2): # solo cuando no hay otra opción
        lspeed, rspeed = -0.5, -0.5
        print("move backwards")
        reading = readings[11]
        current_dir = 0
    # adelante
    elif readings[3] < 0.5 or readings[4] < 0.4 or readings[2] < 0.4 or readings[5] < 0.3 or readings[1] < 0.3: # mirar si se va a chocar
        right, left, current_dir = where_to_go(current_dir, readings) # ver de que sitio se va a chocar

        if left == True: # si nos vamos a chocar en la izquierda hay que mirar a la derecha y muy a la izquierda
            if readings[2] > 0.3: # si un poco a la derecha no hay nada
                lspeed, rspeed = 0, +0.5 # giramos un poco a la derecha
                print("move front a bit right")
                reading = readings[2]
            elif readings[1] > 0.3: # si a la derecha no hay nada
                lspeed, rspeed = -0.2, 0.8 # giramos a la derecha
                print("move front right")
                reading = readings[1]
            elif readings[0] > 0.3: # si muy a la derecha no hay nada
                lspeed, rspeed = -0.4, 1.2 # giramos muy a la derecha
                print("move front much right")
                reading = readings[0]
            elif readings[5] > 0.3: # si a la izquierda no hay nada
                lspeed, rspeed = 0.8, -0.2 # giramos a la izquierda
                print("move front left 1")
                reading = readings[5]
            elif readings[7] > 0.3: # si muy a la izquierda no hay nada
                lspeed, rspeed = -0.4, 1.2 # giramos muy a la izquierda
                print("move front much left 1")
                reading = readings[7]
        else: # si nos vamos a chocar en la derecha hay que mirar a la izquierda y muy a la derecha
            if readings[4] > 0.3: # si un poco a la izquierda no hay nada
                lspeed, rspeed = 0.5, 0 # giramos un poco a la izquierda
                print("move front a bit left")
                reading = readings[4]
            elif readings[5] > 0.3: # si a la izquierda no hay nada
                lspeed, rspeed = 0.8, -0.2 # giramos a la izquierda
                print("move front left")
                reading = readings[5]
            elif readings[7] > 0.3: # si muy a la izquierda no hay nada
                lspeed, rspeed = 1.2, -0.4 # giramos muy a la izquierda
                print("move front much left")
                reading = readings[7]
            elif readings[1] > 0.3: # si a la izquierda no hay nada
                lspeed, rspeed = -0.2, 0.8 # giramos a la derecha
                print("move front right 1")
                reading = readings[1]
            elif readings[0] > 0.3: # si muy a la derecha no hay nada
                lspeed, rspeed = 1.2, -0.4 # giramos muy a la derecha
                print("move front much right 1")
                reading = readings[0]
    else: # si no se va a chocar, que siga recto
        lspeed, rspeed = +0.5, +0.5
        print("move front")
        reading = readings[3]
        current_dir = 0

    return lspeed, rspeed, reading, current_dir


def calculate_acceleration(reading, lspeed, rspeed):
    # Calcula la acceleración solo si va recto, ya que para los giros da problemas hacerlos más rápido, y si iba para atras a veces se movia demasiado para atrás
    if reading != -1:
        # define las variables de entrada
        velocidad = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'velocidad')
        distancia = ctrl.Antecedent(np.arange(0, 1, 0.1), 'distancia')
        aceleracion = ctrl.Consequent(np.arange(0, 2, 0.1), 'aceleracion')

        # define conjuntos difusos
        velocidad['baja'] = fuzz.trimf(velocidad.universe, [0, 0, 0.6])
        velocidad['media'] = fuzz.trimf(velocidad.universe, [0.4, 0.7, 1])
        velocidad['alta'] = fuzz.trimf(velocidad.universe, [0.9, 1.5, 1.5])

        distancia['baja'] = fuzz.trimf(distancia.universe, [0, 0, 0.5])
        distancia['media'] = fuzz.trimf(distancia.universe, [0.25, 0.5, 0.75])
        distancia['alta'] = fuzz.trimf(distancia.universe, [0.5, 1, 1])

        aceleracion['muy_baja'] = fuzz.trimf(aceleracion.universe, [0, 0.2, 0.3])
        aceleracion['baja'] = fuzz.trimf(aceleracion.universe, [0.2, 0.3, 0.5])
        aceleracion['media_baja'] = fuzz.trimf(aceleracion.universe, [0.4, 0.5, 0.7])
        aceleracion['media'] = fuzz.trimf(aceleracion.universe, [0.6, 0.7, 0.9])
        aceleracion['media_alta'] = fuzz.trimf(aceleracion.universe, [1, 1.1, 1.3])
        aceleracion['alta'] = fuzz.trimf(aceleracion.universe, [1.2, 1.3, 1.5])
        aceleracion['muy_alta'] = fuzz.trimf(aceleracion.universe, [1.5, 1.6, 1.8])

        # define reglas difusas
        regla1 = ctrl.Rule(velocidad['baja'] & distancia['alta'], aceleracion['muy_alta']) 
        regla2 = ctrl.Rule(velocidad['baja'] & distancia['baja'], aceleracion['media_baja'])
        regla3 = ctrl.Rule(velocidad['baja'] & distancia['media'], aceleracion['media_alta'])
        regla4 = ctrl.Rule(velocidad['alta'] & distancia['baja'], aceleracion['muy_baja'])
        regla5 = ctrl.Rule(velocidad['alta'] & distancia['media'], aceleracion['media_baja'])
        regla6 = ctrl.Rule(velocidad['alta'] & distancia['alta'], aceleracion['media_alta'])
        regla7 = ctrl.Rule(velocidad['media'] & distancia['media'], aceleracion['media'])
        regla8 = ctrl.Rule(velocidad['media'] & distancia['baja'], aceleracion['baja'])
        regla9 = ctrl.Rule(velocidad['media'] & distancia['alta'], aceleracion['alta'])

        # crea el sistema de control y agregar reglas
        sistema_control = ctrl.ControlSystem([regla1, regla2, regla3, regla4, regla5, regla6, regla7, regla8, regla9])
        controlador = ctrl.ControlSystemSimulation(sistema_control)

        # define las entradas y calcular la salida
        controlador.input['velocidad'] = np.abs(max(lspeed, rspeed)) # daba problemas con negativos, así que se pasa el valor absoluto, y a demás da igual, porque en tema de velocidad, ir a -1.5 y 1.5 es la misma velocidad solo que para delante o para atrás
        controlador.input['distancia'] = reading

        # calcula la salida
        controlador.compute()

        # obteniene el valor de salida
        print(f"Aceleración: {controlador.output['aceleracion']}")

        if lspeed > rspeed:
            ""
        elif rspeed > lspeed:
            ""
        else:
            if rspeed > 0: # solo si va recto suma la aceleración a la velocidad de entrada
                lspeed += controlador.output['aceleracion']
                rspeed += controlador.output['aceleracion']
        #    else:
        #        lspeed -= controlador.output['aceleracion']
        #        rspeed -= controlador.output['aceleracion']

    return lspeed, rspeed


def main(args=None):
    coppelia = robotica_drone.Coppelia()
    coppelia.start_simulation()
    robot = robotica_drone.QuadCopter(coppelia.sim, 'Quadcopter')
    current_dir = 0
    while coppelia.is_running():
        #readings = robot.get_sonar()
        #lspeed, rspeed, reading, current_dir = avoid(readings, current_dir) # calcula hacia dónde ir
        #lspeed, rspeed = calculate_acceleration(reading, lspeed, rspeed) # calcula aceleración
        robot.actuation([0,0,10]) # aplica lo que se ha calculado
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()
