import robotica
import numpy as np
import random
import networkx as nx
import time
import pickle


def detect_obstacles(robot, position, orientation, init_position, init_position_coppelia, goal, goal_coppelia):
    # Cogemos las medidas de los lidars
    readings = robot.get_lidar()

    obstacles = []
    for j in range(0, len(readings[0])):
        if readings[0][j] < 1: # cogemos solo aquellas medidas que representen objetos cercanos (< 1)
            # convertimos la coordenados polares en cartesianas
            x = np.cos(-readings[1][j] + orientation[2]) * readings[0][j] + position[0]
            y = np.sin(-readings[1][j] + orientation[2]) * readings[0][j] + position[1]

            # añadimos los obstáculos a la lista de obstáculos detectados en el sistema de coordenadas del grafo
            obstacles.append(np.round(((goal - init_position) * ((x,y) - init_position_coppelia)) / (goal_coppelia - init_position_coppelia) + init_position))

    return obstacles


def select_init_path(paths, risk, position_drone, goal, objects, objects_per_path, battery_stations_per_path):
    # Usamos decisiones deliverativas para decidir el camino inicial
    
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

    # Finalmente, seleccionamos un camino en base a lo largo que es, los objetos que puede tener y la estaciones de batería que presenta
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
    # se decide si seguir yendo hacia la meta o hacia la estación de carga más cercana

    # calculamos la distancia con todas las estaciones de carga y nos quedamos con la que tenga la distacia más pequeña
    min_path_length_stations = 10000
    counter = 0
    for i in charge_stations:
        path = np.sqrt((position_drone[1] - i[1])**2 + (position_drone[0] - i[0])**2) # no se usa la distancia del grafo que seria lo óptimo debido a que tarda mucho en hacerlo para cada iteración

        if path != None:
            if (path) < min_path_length_stations:
                min_path_length_stations = (path)
                min_shortest_path_stations = path
                min_station = charge_stations[counter]

        counter += 1

    # calculamos la distancia entre el dron y la meta
    path_goal = np.sqrt((position_drone[1] - goal[1])**2 + (position_drone[0] - goal[0])**2)

    # si está más cerca la meta que la estación de carga no tiene ningún sentido ir a la estación de carga
    if (path_goal) <= min_path_length_stations:
        return False, None
    
    # Sino miramos a donde ir mediante un proceso deliverativo
    # De nuevo le damos un valor al riesgo que queremos correr
    if risk == "h":
        risk_number = 0.5
    elif risk == "m":
        risk_number = 0.75
    else:
        risk_number = 0.9

    # Y penalizamos más o menos que estemos en un camino con muchos o con pocos obstáculos
    objects_in_this_path *= objects * risk_number

    # Calculamos cuanta batería nos quedaría cuando llegaramos a la meta y usamos el riesgo de tal manera que un riesgo alto asume que pierde un número cercano a 1% de batería en cada paso, y uno bajo que pierde un número cercano a 0% de batería en cada paso
    battery_left_to_reach_goal = battery - ((path_goal / 2.5) * risk_number + objects_in_this_path) * 1

    # Finalmente usamos el riesgo para valorar si la batería que nos quedará al llegar a la meta/estación de carga es suficiente para agantar un poco más o si es mejor y yendo ya
    if battery_left_to_reach_goal > 3 * risk_number:
        return False, None
    else:
        return True, min_station


def main(args=None):
    # Definimos la altitud mínima
    minimum_altitude = 0.150

    # Definimos las variables con las posiciones en las que se encuentran las estaciones de carga, la posición inicial y la meta
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
        print("Today is weekend")
        objects = 0.3
    else:
        print("Today is not weekend")
        objects = 0.1

    if is_holiday == 1:
        print("Today is holidays")
        objects += 0.3
    else:
        print("Today is not holiday")
        objects += 0.1

    if is_raining == 1:
        print("Today is raining")
        objects += 0.3
    else:
        print("Today is not raining")
        objects += 0.1

    total_collidable_objects = int(round(total_map_objects * objects))

    obj_sequence = list(range(total_map_objects))

    selected_objects = random.sample(obj_sequence, total_collidable_objects) # seleccionamos al azar los objetos que se activaran

    # Le preguntamos al usuario que riesgo quiere asumir
    print('Which is the risk that you want to take? (h=high, m=medium, l=low)')
    risk = input().lower()

    # Cargamos los caminos y el mapa
    paths = []

    with open('D:/graph.pkl', 'rb') as file:
        graph = pickle.load(file)

    with open('D:/path_high.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    with open('D:/path_mid.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    with open('D:/path_low.pkl', 'rb') as file:
        paths.append(pickle.load(file))

    # iniciamos coppelia, el dron y empezamos la simulación
    coppelia = robotica.Coppelia()
    robot = robotica.QuadCopter(coppelia.sim, 'Quadcopter')
    coppelia.start_simulation()

    # hacemos que los objetos seleccionados sean no colisionables y invisibles
    robot.desactivate_objects(selected_objects)

    # Seleccionamos el camino que seguiremos usando decisiones deliverativas
    selected_path, objects_in_this_path = select_init_path(paths, risk, init_position, goal, objects, objects_per_path, battery_stations_per_path)

    # para saber el tiempo medio de cada camino, calcularemos el tiempo que se ha tardado en llegar a la meta
    start_time = time.time()

    current_goal = goal
    
    # una vez tenemos todos los valores iniciales empezamos a mover el dron
    while coppelia.is_running():
        try:
            identifier = 0
            # cogemos la posición y orientación del dron y la transformamos al sistema de coordenadas del grafo
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
                    identifier = 2

            # si hace falta cargarlo
            if need_to_charge == True:
                # ponemos como meta la estación de carga
                current_goal = selected_charge_station

                # si estamos en la estación de carga
                if np.abs(position[0] - selected_charge_station[0]) < 10 and np.abs(position[1] - selected_charge_station[1]) < 10:
                    altitude = position_coppelia[2] # guardamos la altura actual
                
                    robot.setposition_target([position_coppelia[0], position_coppelia[1], minimum_altitude]) # decendemos

                    time.sleep(3) # tardamos 3 segundos en subir simulando que se está cargando el dron

                    position_coppelia, orientation = robot.getposition_drone()

                    battery = 100 # ya se ha cargado la batería
                    robot.setposition_target([position_coppelia[0], position_coppelia[1], altitude]) # volvemos a la altitud anterior
                    
                    position_coppelia, orientation = robot.getposition_drone()

                    need_to_charge = False
                    current_goal = goal # la meta vuelve a ser la del principio
                    identifier = 2

            # si se ha llegado a la meta acabamos la simulación
            if round(position_coppelia[0], 0) == round(goal_coppelia[0], 0) and round(position_coppelia[1], 0) == round(goal_coppelia[1], 0):
                print("Game finished: Congratulations, you have reached the goal in", round(time.time() - start_time,2), "seconds :)")
                break
            
            # detectamos los objetos que esten cerca de nuestro dron
            detected_obstacles = detect_obstacles(robot, position_coppelia, orientation, init_position, init_position_coppelia, goal, goal_coppelia)
            
            for node in detected_obstacles:
                nodes = []
                # para cada objeto se mira si el nodo está ya quitado del grafo (si ya está detectado no hace falta borrarlo de nuevo ni actualizar la ruta)
                if graph.has_node((node[0], node[1])):
                    if identifier == 0:
                        identifier = 1
                    
                    # si no está quitado lo quitamos, y quitamos los vecinos para simular el tamaño del dron
                    graph.remove_node((node[0], node[1]))

                    for j in range(1, 5):
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
                        
            # si se ha quitado algun nodo o se ha decidido cambiar la meta se recalcula la ruta con D*
            if identifier >= 1:
                try:
                    selected_path = nx.shortest_path(graph, source=(position[0], position[1]), target=(current_goal[0], current_goal[1]))
                except:
                    # si no encuentra la meta exactamente se coge algún nodo cercano a la meta
                    distances = nx.single_source_shortest_path_length(graph, (current_goal[0], current_goal[1]))

                    closest_node = min(distances, key=distances.get)

                    try:
                        selected_path = nx.shortest_path(graph, source=(position[0], position[1]), target=closest_node)
                    except:
                        if identifier == 2:
                            try:
                                # si no encuentra el punto de salido se coge algún nodo cercano a este
                                distances = nx.single_source_shortest_path_length(graph, (position[0], position[1]))

                                # Obtener el nodo más cercano (el nodo con la menor distancia)
                                closest_node_2 = min(distances, key=distances.get)

                                selected_path = nx.shortest_path(graph, source=closest_node_2, target=closest_node)
                            except:
                                pass
                        else:
                            pass
            
            # se actualiza la posición del dron
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
                time.sleep(3)
                print("Game over: You have run out of battery :(")
                break

            robot.setposition_target([position_target[0], position_target[1], 0.5])

            try:
                selected_path = selected_path[2:]
            except:
                pass

            # En cada paso que da el robot le bajamos un poco la batería de manera aleatoria
            battery -= round(random.random(), 3)
            print(battery)
        except:
            break
    
    # volvemos todos los objetos visibles y colisionables y acabamos la simulación
    robot.activate_objects_again()
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()