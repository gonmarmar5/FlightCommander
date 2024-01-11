import robotica_drone_2_copy as robotica
import numpy as np
import random
import csv
import networkx as nx
import time
from PIL import Image
from queue import PriorityQueue
import math
import pickle
import matplotlib.pyplot as plt

def heuristics(node, goal, distance_metric='euclidean'):
        if distance_metric == 'manhattan':
            return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
        elif distance_metric == 'diagonal':
            dx = abs(node[0] - goal[0])
            dy = abs(node[1] - goal[1])
            return max(dx, dy)
        elif distance_metric == 'octile':
            dx = abs(node[0] - goal[0])
            dy = abs(node[1] - goal[1])
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        else:
            return np.linalg.norm(np.array(node) - np.array(goal))
    
def reconstruct_path(came_from, start, goal):
    current_node = goal
    path = []

    while current_node != start:
        path.append(current_node)
        current_node = came_from[current_node]

    path.append(start)
    return path[::-1]

def astar(graph, start, goal, heuristic):
    open_set = PriorityQueue()
    start = tuple(start)
    goal = tuple(goal)
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}
    visited_nodes = set()

    while not open_set.empty():
        current_cost, current_node = open_set.get()
        visited_nodes.add(current_node) # Guarda los nodos que ha visitado

        if current_node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, visited_nodes # Devuelve el camino

        for next_node in list(graph.neighbors(current_node)):
            try:
                new_cost = cost_so_far[current_node] + graph[current_node][next_node]["weight"]
            except:
                new_cost = cost_so_far[current_node] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]: # Itera dando prioridad primero a los nodos que minimizan una heurística la cual valora como de cerca queda la meta y que tienen un coste menor desde la posición inicial
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristics(next_node, goal, heuristic)
                open_set.put((priority, next_node))
                came_from[next_node] = current_node

    return None, visited_nodes  # No hay camino entre start y goal


def detect_obstacles(robot, position, orientation, init_position, init_position_coppelia, goal, goal_coppelia):
    # Cogemos las medidas de los lidars
    readings = robot.get_lidar()

    obstacles = []
    for j in range(0, len(readings[0])):
        if readings[0][j] < 1: # cogemos solo aquellas medidas que representen objetos cercanos (< 0.5)
            # convertimos la coordenados polares en cartesianas
            x = np.cos(-readings[1][j] + orientation[2]) * readings[0][j] + position[0]
            y = np.sin(-readings[1][j] + orientation[2]) * readings[0][j] + position[1]

            # añadimos los obstáculos a la lista de obstáculos detectados
            obstacles.append(np.round(((goal - init_position) * ((x,y) - init_position_coppelia)) / (goal_coppelia - init_position_coppelia) + init_position))

    return obstacles


def select_init_path(paths, risk, position_drone, goal, objects, objects_per_path, battery_stations_per_path):
    # Usamos decisiones deliverativas para saber a donde vamos
    
    # Cogemos cuanto de largo es cada camino
    length_per_path = [len(paths[0]), len(paths[1]), len(paths[2])]

    # Primero le atribuimos un valor a cada riesgo
    if risk == "h":
        risk_number = 0.5
    elif risk == "m":
        risk_number = 1
    else:
        risk_number = 1.5
        

    # Penalizamos más o menos según el riesgo lo que tardaremos en esquivar todos los objetos que encontremos (que es una cantidad desconocida ya que sabemos cuantos objetos se activaran y cuantos objectos hay en cada camino pero no cuantos objetos se activaran en cada camino)
    battery_stations_per_path_2 = []
    for i in range(len(objects_per_path)):
        objects_per_path[i] *= objects * risk_number / 1.5

        # Hacemos lo mismo pero para las estaciones batería
        battery_stations_per_path_2.append((max(battery_stations_per_path) - battery_stations_per_path[i]) * risk_number)

        length_per_path[i] = length_per_path[i] / 44 / risk_number

    # Finalmente, seleccionamos un camino en base a lo largo que es (independiente del riesgo) los objetos que puede tener (dependiente del riesgo) y la estaciones de batería que presenta (dependiente del riesgo)
    risks = -np.array(length_per_path) - np.array(objects_per_path) - np.array(battery_stations_per_path_2)
    selected_path = np.argmax(risks)

    if selected_path == 0:
        print("The algorithm has selected the difficult path")
    elif selected_path == 1:
        print("The algorithm has selected the medium path")
    else:
        print("The algorithm has selected the easy path")

    return paths[selected_path], objects_per_path[selected_path]


def select_path(graph, risk, charge_stations, position_drone, goal, objects, objects_in_this_path, battery):
    # calculamos la distancia con todas las estaciones de carga y nos quedamos con la que tenga la distacia más pequeña
    min_path_length_stations = 10000
    counter = 0
    for i in charge_stations:
        #path, visited_nodes = astar(graph, position_drone, i, "euclidean") # Calculamos la distancia entre la posición del dron y cada estación de carga
        #path = nx.shortest_path(graph, source=(position_drone[0], position_drone[1]), target=(i[0], i[1]))
        path = np.sqrt((position_drone[1] - i[1])**2 + (position_drone[0] - i[0])**2)

        if path != None: # Nos quedamos con la que esté más cerca de nuestra posición
            if (path) < min_path_length_stations:
                min_path_length_stations = (path)
                min_shortest_path_stations = path
                min_station = charge_stations[counter]

        counter += 1

    # calculamos la distancia entre el dron y la meta
    #path_goal, visited_nodes = astar(graph, position_drone, goal, "euclidean")
    #path_goal = nx.shortest_path(graph, source=(position_drone[0], position_drone[1]), target=(goal[0], goal[1]))
    #path_goal = len(path_goal)
    path_goal = np.sqrt((position_drone[1] - goal[1])**2 + (position_drone[0] - goal[0])**2)

    # si está más cerca la meta que la estación de carga no tiene ningún sentido ir a la estación de carga
    if (path_goal) <= min_path_length_stations:
        return False, None
    
    # Sino miramos a donder mediante un proceso deliverativo
    # De nuevo le damos un valor al riesgo que queremos correr
    if risk == "h":
        risk_number = 0.5
    elif risk == "m":
        risk_number = 0.75
    else:
        risk_number = 1

    # Y penalizamos más o menos que estemos en un camino con muchos o con pocos obstáculos
    objects_in_this_path *= objects * risk_number

    # Calculamos cuanta batería nos quedaría cuando llegaramos a la meta o a la estación más cercana y usamos el riesgo de tal manera que un riesgo alto asume que solo pierde un 1% de batería en cada paso, y uno bajo que pierde un 2% de batería en cada paso
    battery_left_to_reach_nearest_station = battery - ((min_path_length_stations / 2) * (risk_number + 0.3) + objects_in_this_path) * 1
    battery_left_to_reach_goal = battery - ((path_goal / 2.5) * risk_number + objects_in_this_path) * 1

    # Finalmente usamos el riesgo para valorar si la batería que nos quedará al llegar a la meta/estación de carga es suficiente para agantar un poco más o si es mejor y yendo ya
    if battery_left_to_reach_goal > 3 * risk_number:
        return False, None
    else:
        return True, min_station


def main(args=None):
    # Definimos la altitud
    maximum_altitude = 2.4
    minimum_altitude = 0.150

    # Definimos las variables de la dirección
    current_direction = "front"
    path_direction = "front"

    # Definimos las variables de las posiciones de carga y meta
    goal = np.array([288, 211])
    goal_coppelia = np.array([3.2, 0.55])

    init_position = np.array([49, 119])
    init_position_coppelia = np.array([-10.925, -4.75])

    charge_stations_coppelia = [[-5.425, 5.9],
                    [-9.65, 4.225], 
                    [1.35, -9.3],
                    [6.55, 9.025],
                    [-10, -8.5]]
    
    charge_stations = []
    for charge_station in charge_stations_coppelia:
        position = np.round(((goal - init_position) * (charge_station - init_position_coppelia)) /  (goal_coppelia - init_position_coppelia) + init_position)
        charge_stations.append(position)

    # Definimos las variables de la batería
    need_to_charge = False
    battery = 100
    battery_stations_per_path = [1,2,3]

    # Definimos las variables de los objetos
    total_map_objects = 25
    objects_per_path = [16, 13, 10]
    is_weekend = random.randint(0,1)
    is_holiday = random.randint(0, 1)
    is_raining = random.randint(0,1)

    # Definimos el número de objetos que se activaran dependiendo de si llueve y de si es fin de semana o vacaciones
    if is_weekend == 1:
        print("Today is holidays")
        objects = 0.3
    else:
        print("Today is not holiday")
        objects = 0.1

    if is_holiday == 1:
        print("Today is holidays")
        objects = 0.3
    else:
        print("Today is not holiday")
        objects = 0.1

    if is_raining == 1:
        print("Today is raining")
        objects += 0.3
    else:
        print("Today is not raining")
        objects += 0.1

    total_collidable_objects = int(round(total_map_objects * objects))

    obj_sequence = list(range(total_map_objects))

    selected_objects = random.sample(obj_sequence, total_collidable_objects) # seleccionamos al azar los objetos que se activaran

    # Le preguntamos al usuario que riesgo quiere tomar
    print('Which is the risk that you want to take? (h=high, m=medium, l=low)')
    risk = input().lower()

    # Cargamos los caminos y el mapa
    paths = []

    with open('D:/graph_4.pkl', 'rb') as file:
        graph = pickle.load(file)

    with open('D:/path_mid_3.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    with open('D:/path_high_3.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    with open('D:/path_low_3.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    # iniciamos coppelia, el robot y empezamos la simulación
    coppelia = robotica.Coppelia()
    robot = robotica.QuadCopter(coppelia.sim, 'Quadcopter')
    coppelia.start_simulation()

    # hacemos que los objetos seleccionados esten collidable y detectable
    robot.desactivate_objects(selected_objects)

    # Seleccionamos el camino que seguiremos usando decisiones deliverativas
    selected_path, objects_in_this_path = select_init_path(paths, risk, init_position, goal, objects, objects_per_path, battery_stations_per_path)

    # como al final queremos llegar en el mínimo tiempo posible cogemos el tiempo al empezar la simulación
    start_time = time.time()

    current_goal = goal
    
    # una vez tenemos todos los valores iniciales empezamos a mover el robot
    while coppelia.is_running():
        identifier = 0
        # actualizamos la posición y orientación del dron
        position_coppelia, orientation = robot.getposition_drone()

        position_coppelia = np.array(position_coppelia)

        position = np.round(((goal - init_position) * (position_coppelia[:2] - init_position_coppelia)) /  (goal_coppelia - init_position_coppelia) + init_position)

        # actualizamos que camino queremos seguir (en función a si necesitamos cargar el dron o no)
        if need_to_charge == False and battery < 80:
            try:
                need_to_charge, selected_charge_station = select_path(graph, risk, charge_stations, position, goal, objects, objects_in_this_path, battery)
            except:
                need_to_charge = False
                
            if need_to_charge == True:
                identifier = 1
        # si hace falta cargarlo
        if need_to_charge == True:
            # cambiamos la dirección que estamos siguiendo
            current_goal = selected_charge_station

            # si estamos en la estación de carga
            if np.abs(position[0] - selected_charge_station[0]) < 5 and np.abs(position[1] - selected_charge_station[1]) < 5:
                altitude = position_coppelia[2] # guardamos la altura actual
            
                robot.setposition_target([position_coppelia[0], position_coppelia[1], minimum_altitude]) # decendemos

                while True:
                    position_coppelia, orientation = robot.getposition_drone()

                    if round(position_coppelia[2]) == minimum_altitude: # vamos mirando si el dron ha llegado abajo
                        battery = 100
                        robot.setposition_target([position_coppelia[0], position_coppelia[1], altitude]) # cuando llega definimos que queremos volver a la altitud anterior y que se ha cargado la batería
                        break
                
                while True:
                    position_coppelia, orientation = robot.getposition_drone()

                    if round(position_coppelia[2]) == altitude: # miramos si ya ha llegado a la actitud deseada
                        need_to_charge = False
                        current_goal = goal
                        break

        if round(position_coppelia[0], 0) == round(goal_coppelia[0], 0) and round(position_coppelia[1], 0) == round(goal_coppelia[1], 0):
            print("Game finished: Congratulations, you have reached the goal in", time.time() - start_time, "seconds :)")
            break
        
        detected_obstacles = detect_obstacles(robot, position_coppelia, orientation, init_position, init_position_coppelia, goal, goal_coppelia)

        for node in detected_obstacles:
            nodes = []
            if graph.has_node((node[0], node[1])):
                identifier = 1
                graph.remove_node((node[0], node[1]))

                for j in range(1, 11):
                    nodes.append((node[0] + j, node[1]))
                    nodes.append((node[0] - j, node[1]))
                    nodes.append((node[0], node[1] + j))
                    nodes.append((node[0], node[1] - j))
                    nodes.append((node[0] + j, node[1] + j))
                    nodes.append((node[0] + j, node[1] - j))
                    nodes.append((node[0] - j, node[1] + j)) 
                    nodes.append((node[0] - j, node[1] - j))

            for i in nodes:
                try:
                    graph.remove_node(i)
                except:
                    pass

        if identifier == 1:
            try:
                selected_path = nx.shortest_path(graph, source=(position[0], position[1]), target=(current_goal[0], current_goal[1]))
            except:
                distances = nx.single_source_shortest_path_length(graph, (current_goal[0], current_goal[1]))

                # Obtener el nodo más cercano (el nodo con la menor distancia)
                closest_node = min(distances, key=distances.get)

                try:
                    selected_path = nx.shortest_path(graph, source=(position[0], position[1]), target=closest_node)
                except:
                    pass
            
        try:
            position_target = ((goal_coppelia - init_position_coppelia) * (selected_path[10] - init_position)) / (goal - init_position) + init_position_coppelia
        except:
            try:
                position_target = ((goal_coppelia - init_position_coppelia) * (selected_path[5] - init_position)) / (goal - init_position) + init_position_coppelia
            except:
                position_target = ((goal_coppelia - init_position_coppelia) * (selected_path[0] - init_position)) / (goal - init_position) + init_position_coppelia

        # Si la batería es muy baja aterriza y la simulación ha terminado
        if battery <= 1:
            robot.setposition_target([position_coppelia[0], position_coppelia[1], minimum_altitude])
            print("Game over: You have run out of battery :(")
            break

        robot.setposition_target([position_target[0], position_target[1], 0.5])

        try:
            selected_path = selected_path[2:]
        except:
            pass

        # En cada paso que da el robot le bajamos un poco la batería de forma random
        battery -= round(random.random(), 3)
        print(battery)
    
    robot.activate_objects_again()
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()