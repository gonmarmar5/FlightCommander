import robotica_drone_2 as robotica
import numpy as np
import random
import csv
import networkx as nx
import time


def move_drone(robot, current_direction, path_direction, position, maximum_altitude, identifier):
    # Cogemos las medidas de los lidars
    readings = robot.get_lidar(current_direction)

    objects = []
    current_object = 0
    objects_2 = 0
    for i in range(0, 2):
        for j in range(0, len(readings[i][0])):
            if readings[i][0][j] < 0.5 and (i == 0 or (readings[i][1][j] >= 60 / 180 * np.pi and readings[i][1][j] >= -60 / 180 * np.pi)): # cogemos solo aquellas medidas que representen objetos lejanos (> 0.5) y que no esten repetidas en otro lidar
                # convertimos la coordenados polares en cartesianas
                if i == 0:
                    x = np.cos(-readings[i][1][j]) * readings[i][0][j] + position[0]
                    y = np.sin(-readings[i][1][j]) * readings[i][0][j] + position[1]
                else:
                    x = np.cos(-readings[i][1][j]) * readings[i][0][j] + position[0]
                    y = np.sin(-readings[i][1][j]) * readings[i][0][j] + position[1]

                try:
                    # Si la medida que representa un objeto es similar a la siguienta hablan del mismo objeto así que los juntamos
                    if np.abs(objects[current_object][objects_2-1][0] - x) < 0.2 and np.abs(objects[current_object][objects_2-1][1] - y) < 0.2:
                        objects[current_object].append([x,y])
                        objects_2 += 1
                    else:
                        objects.append([[x,y]])
                        objects += 1
                        objects_2 = 0
                except:
                    objects.append([[x, y]])
                    objects_2 = 0

    # como es un circulo, el primer y último objeto pueden ser el mismo así que si ese es el caso, los juntamos
    if np.abs(objects[0][0][0] - objects[-1][-1][0]) < 0.2 and np.abs(objects[0][0][1] - objects[-1][-1][1]) < 0.2:
        objects.append(objects[0] + objects[-1])
        objects.remove(0)
        objects.remove(-1)

    # miramos si nos podemos chocar yendo hacia alguna dirección
    directions = {}
    objects_directions = {}
    objects_directions["right"] = False
    objects_directions["left"] = False
    objects_directions["front"] = False
    objects_directions["back"] = False
    for i in objects:
        if (min(i[:][0]) >= position[0] - 0.2 and min(i[:][0]) <= position[0] + 0.2) or (max(i[:][0]) >= position[0] - 0.2 and max(i[:][0]) <= position[0] + 0.2) or position[0] - 0.2 >= min(i[:][0]) and position[0] + 0.2 <= max(i[:][0]):
            if position[1] > min(i[:][1]):
                directions["back"] = True
                objects_directions["back"].append(i)

            else:
                directions["front"] = True
                objects_directions["front"].append(i)

        if (min(i[:][1]) >= position[1] - 0.2 and min(i[:][1]) <= position[1] + 0.2) or (max(i[:][1]) >= position[1] - 0.2 and max(i[:][1]) <= position[1] + 0.2) or position[1] - 0.2 >= min(i[:][1]) and position[1] + 0.2 <= max(i[:][1]):
            if position[1] > min(i[:][0]):
                directions["left"] = True
                objects_directions["left"].append(i)
            else:
                directions["right"] = True
                objects_directions["right"].append(i)

    # Si no estamos yendo en la dirección del camino que se ha seleccionado y esta está libre vamos en esta dirección
    if current_direction != path_direction and identifier == 0:
        if objects_directions[path_direction] == False:
            current_direction = path_direction

    else:
        if objects_directions[current_direction] == True: # Sino lo está pero en la dirección en la que ya estabamos yendo si que lo está, seguimos en está dirección
            length = {}
            for i in objects_directions[current_direction]: # Si tampoco lo está, primero miramos las opciones de los lados
                if current_direction == "front" or current_direction == "back":
                    if directions["left"] == False:
                        length["left"] = np.abs(min(objects_directions[current_direction][:][0]) - position[0])
                    if directions["right"] == False:
                        length["right"] = np.abs(max(objects_directions[current_direction][:][0]) - position[0])
                else:
                    if directions["front"] == False:
                        length["front"] = np.abs(max(objects_directions[current_direction][:][1]) - position[1])
                    if directions["back"] == False:
                        length["back"] = np.abs(min(objects_directions[current_direction][:][1]) - position[1])

            objects = []
            current_object = 0
            objects_2 = 0
            for j in range(0, len(readings[2][0])): # Y estimamos si se puede ir por arriba o por abajo igual que antes hemos hecho con los lados
                if readings[i][0][j] < 0.5:
                    if i == 0:
                        z = np.sin(-readings[i][1][j]) * readings[i][0][j] + position[2]
                    else:
                        z = np.sin(-readings[i][1][j]) * readings[i][0][j] + position[2]

                    try:
                        if np.abs(objects[current_object][objects_2-1] - z) < 0.2:
                            objects[current_object].append(z)
                            objects_2 += 1
                        else:
                            objects.append([z])
                            objects += 1
                            objects_2 = 0
                    except:
                        objects.append([z])
                        objects_2 = 0


            objects_directions = {}
            for i in objects:
                if (min(i[:]) >= position[2] - 0.2 and min(i[:]) <= position[2] + 0.2) or (max(i[:]) >= position[2] - 0.2 and max(i[:]) <= position[2] + 0.2) or position[2] - 0.2 >= min(i[:]) and position[2] + 0.2 <= max(i[:]):
                    if position[2] > min(i[:]):
                        objects_directions["up"].append(i)


            for i in objects_directions["up"]:
                if (max(objects_directions["up"]) != maximum_altitude) and (min(objects_directions["up"]) != 0):
                    length["down"] = np.abs(min(objects_directions["up"][:]) - position[2])
                    length["up"] = np.abs(max(objects_directions["up"][:]) - position[2])

            # Finalmente si todas las direcciones están bloqueadas volvemos hacia atrás
            if min(objects_directions["up"][:]) == 0 and max(objects_directions["up"][:]) == maximum_altitude and (((current_direction == "front" or current_direction == "back") and directions["left"] == True and directions["right"] == True) or ((current_direction == "left" or current_direction == "right") and directions["front"] == True and directions["back"] == True)):
                if current_direction == "front":
                    current_direction = "back"
                elif current_direction == "back":
                    current_direction = "front"
                elif current_direction == "left":
                    current_direction = "right"
                else:
                    current_direction = "left"

                identifier = 1 # para evitar entrar en un bucle si entramos en una posición sin salida siguiendo el camino ideal

            else: # Y sino seleccionamos con la que menos tenemos que desplazarnos
                current_direction_2 = min(length, key=length.get)

                if current_direction_2 != "up" or current_direction_2 != "down":
                    current_direction = current_direction_2
                
                identifier = 0
    
    # nos desplazamos en esta dirección
    position_target = robot.getposition_target()
    
    if current_direction_2 == "up":
        position_target[2] += 0.1
    elif current_direction_2 == "down":
        position_target[2] -= 0.1
    elif current_direction == "front":
        position_target[0] += 0.2
    elif current_direction == "back":
        position_target[0] -= 0.2
    elif current_direction == "left":
        position_target[1] -= 0.2
    elif current_direction == "right":
        position_target[1] += 0.2

    return current_direction


def select_init_path(maps_, risk, position_drone, goal, objects, objects_per_path, battery_stations_per_path):
    # definimos las variables donde guardar los graphs
    graphs = []
    shortest_paths = []
    length_per_path = []
    counter = 0
    
    # Creamos un graph para cada mapa
    for map_ in maps_:
        G = nx.Graph()

        rows, cols = map_.shape

        for i in range(rows):
            for j in range(cols):
                if map_[i, j] == 0:
                    G.add_node((i, j))

                    neighbors = [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)]
                    for neighbor in neighbors:
                        ni, nj = neighbor
                        if 0 <= ni < rows and 0 <= nj < cols and map_[ni, nj] == 0:
                            G.add_edge((i, j), neighbor)

        graphs.append(G)

        # calculamos dijkstra para sacar el camino a seguir más óptimo para cada mapa
        if counter != 3: # para el mapa completo ya se calculará cada vez
            shortest_path_goal, path_length_goal = nx.single_source_dijkstra(G, source=position_drone, target=goal)
            shortest_paths.append(shortest_path_goal)
            length_per_path.append(path_length_goal)

        counter += 1

    # Usamos decisiones deliverativas para saber a donde vamos
    # Primero le atribuimos un valor a cada riesgo
    if risk == "h":
        risk_number = 0.5
    elif risk == "m":
        risk_number = 1
    else:
        risk_number = 1.5

    # Penalizamos más o menos según el riesgo lo que tardaremos en esquivar todos los objetos que encontremos (que es una cantidad desconocida ya que sabemos cuantos objetos se activaran y cuantos objectos hay en cada camino pero no cuantos objetos se activaran en cada camino)
    objects_per_path *= objects * risk_number

    # Hacemos lo mismo pero para las estaciones batería
    battery_stations_per_path *= (max(battery_stations_per_path) - battery_stations_per_path) * risk_number

    print(objects_per_path)
    print(battery_stations_per_path)

    # Finalmente, seleccionamos un camino en base a lo largo que es (independiente del riesgo) los objetos que puede tener (dependiente del riesgo) y la estaciones de batería que presenta (dependiente del riesgo)
    selected_path = np.argmax(-length_per_path - objects_per_path - battery_stations_per_path)

    return shortest_paths[selected_path], graphs[3], objects_per_path[selected_path]


def select_path(G, risk, charge_stations, position_drone, goal, objects, objects_in_this_path, battery):
    # calculamos la distancia con todas las estaciones de carga y nos quedamos con la que tenga la distacia más pequeña
    min_path_length_stations = 1000
    counter = 0
    for i in charge_stations:
        shortest_path, path_length = nx.single_source_dijkstra(G, source=position_drone, target=i)

        if path_length < min_path_length_stations:
            min_path_length_stations = path_length
            min_shortest_path_stations = shortest_path
            min_station = charge_stations[counter]

        counter += 1

    # calculamos la distancia entre el dron y la meta
    shortest_path_goal, path_length_goal = nx.single_source_dijkstra(G, source=position_drone, target=goal)

    # si está más cerca la meta que la estación de carga no tiene ningún sentido ir a la estación de carga
    if path_length_goal <= min_path_length_stations:
        return None, False, None
    
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
    battery_left_to_reach_nearest_station = battery - ((min_path_length_stations + objects_in_this_path) / (2 / risk_number))
    battery_left_to_reach_goal = battery - ((path_length_goal + objects_in_this_path) / (2 / risk_number))

    # Finalmente usamos el riesgo para valorar si la batería que nos quedará al llegar a la meta/estación de carga es suficiente para agantar un poco más o si es mejor y yendo ya
    if battery_left_to_reach_goal < 10 * risk_number:
        return None, False, None
    elif battery_left_to_reach_nearest_station < 5 * risk_number:
        return min_shortest_path_stations, True, min_station
    else:
        return None, False, None



def main(args=None):
    # Definimos la altitud máxima
    maximum_altitude = 4

    # Definimos las variables de la dirección
    current_direction = "front"
    path_direction = "front"

    # Definimos las variables de las posiciones de carga y meta
    goal = [[13, 15]]
    charge_stations = [[10, 7],
                    [5, 6], 
                    [9, 8]]
    
    # Definimos las variables de la batería
    need_to_charge = False
    battery = 100
    battery_stations_per_path = [0,1,2]

    # Definimos las variables de los objetos
    total_map_objects = 50
    objects_per_path = [10, 5, 3]
    is_weekend = random.randint(0,6)
    if is_weekend == 0 or is_weekend == 1:
        is_weekend = 1
    else:
        is_weekend = 0
    is_holiday = random.randint(0, 9)
    if is_holiday != 1:
        is_holiday = 0
    is_raining = random.randint(0,1)

    # Definimos el número de objetos que se activaran dependiendo de si llueve y de si es fin de semana o vacaciones
    if is_weekend == 1 or is_holiday == 1:
        print("Today is holidays")
        objects = 0.3
    else:
        print("Today is not holiday")
        objects = 0.5

    if is_raining == 1:
        print("Today is raining")
        objects += 0.5
    else:
        print("Today is not raining")
        objects += 0.3

    total_collidable_objects = int(round(total_map_objects * objects))

    selected_objects = random.sample(total_map_objects, total_collidable_objects) # seleccionamos al azar los objetos que se activaran

    # Le preguntamos al usuario que riesgo quiere tomar
    print('Which is the risk that you want to take? (h=high, m=medium, l=low)')
    risk = input().lower()

    # Cargamos los mapas (para hacerlo más fácil uno para cada camino)
    maps = []
    for i in range(1, 5):
        map_ = []
        with open(f'grid_map_{i}.csv', 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                map_.append([int(cell) for cell in row])
                
        print(map_)
        maps.append(map_)
    
    # Seleccionamos el camino que seguiremos usando decisiones deliverativas
    landmarks, G, objects_in_this_path = select_init_path(map_, risk, position, goal, objects, objects_per_path, battery_stations_per_path)


    # iniciamos coppelia, el robot y empezamos la simulación
    coppelia = robotica.Coppelia()
    robot = robotica.QuadCopter(coppelia.sim, 'Quadcopter')
    coppelia.start_simulation()

    # hacemos que los objetos seleccionados esten collidable y detectable
    robot.activate_objects(selected_objects)

    identifier = 0

    # como al final queremos llegar en el mínimo tiempo posible cogemos el tiempo al empezar la simulación
    start_time = time.time()
    
    # una vez tenemos todos los valores iniciales empezamos a mover el robot
    while coppelia.is_running():
        # actualizamos la posición y orientación del dron
        position, orientation = robot.getposition_drone()

        # actualizamos que camino queremos seguir (en función a si necesitamos cargar el dron o no)
        if need_to_charge == False:
            landmarks_2, need_to_charge, selected_charge_station = select_path(G, risk, charge_stations, position, goal, objects, objects_in_this_path, battery)

        # si hace falta cargarlo
        if need_to_charge == True:
            # cambiamos la dirección que estamos siguiendo
            current_landmarks = landmarks_2

            # si estamos en la estación de carga
            if round(position[:2], 0) == selected_charge_station:
                altitude = position[2] # guardamos la altura actual
            
            robot.setposition_target([position[0], position[1], 0]) # decendemos

            while True:
                position, orientation = robot.getposition_drone()

                if round(position[2]) == 0: # vamos mirando si el dron ha llegado abajo
                    battery = 100
                    robot.setposition_target([position[0], position[1], altitude]) # cuando llega definimos que queremos volver a la altitud anterior y que se ha cargado la batería
                    break
            
            while True:
                position, orientation = robot.getposition_drone()

                if round(position[2]) == altitude: # miramos si ya ha llegado a la actitud deseada
                    need_to_charge = False
                    current_landmarks = landmarks # y si ha llegado volvemos al camino original y volvemos a suponer que no hace falta cargar la batería
                    break
        
        # uncomment try and except when the function has been checked
        #try:
        dist = 10000
        # para actualizar la posición a la que queremos ir para seguir con el camino definido al principio
        for i in range(len(current_landmarks), 0, -1):
            if round(current_landmarks[i][0] - position[0], 0) == 0 and round(current_landmarks[i][1] - position[1], 0) == 0: # miramos si el dron está en el camino y si eso, pues quiere ir al siguiento punto del camino
                if np.abs(current_landmarks[i - 1][0] - position[0]) > np.abs(current_landmarks[i - 1][1]):
                    if current_landmarks[i - 1][0] - position[0] > 0:
                        path_direction = "front"
                    else:
                        path_direction = "back"
                else:
                    if current_landmarks[i - 1][1] - position[1] > 0:
                        path_direction = "right"
                    else:
                        path_direction = "left"

            
            if np.sqrt((current_landmarks[i][0] - position[0], 0) ** 2 + (current_landmarks[i][0] - position[0], 0) ** 2) < dist: # miramos si el dron se ha salido del camino y si eso se dirige al punto del camino que le pilla más cerca
                if np.abs(current_landmarks[i - 1][0] - position[0]) > np.abs(current_landmarks[i - 1][1]):
                    if current_landmarks[i - 1][0] - position[0] > 0:
                        path_direction = "front"
                    else:
                        path_direction = "back"
                else:
                    if current_landmarks[i - 1][1] - position[1] > 0:
                        path_direction = "right"
                    else:
                        path_direction = "left"
            else:
                dist = np.sqrt((current_landmarks[i][0] - position[0], 0) ** 2 + (current_landmarks[i][0] - position[0], 0) ** 2)
        #except:
        #    print("Game finished: Destination Reached. Time: ", time.time() - start_time) # si peta la función es porque ya está en el punto final, así que hemos acabado
        #    break
    
        # si no ha petado todavía no hemos acabado y por lo tanto hay que actualizar la posición del dron
        current_direction = move_drone(robot, current_direction, path_direction, position, maximum_altitude, identifier)

        # En cada paso que da el robot le bajamos un poco la batería de forma random
        battery -= random.random() * 2

        # Si el dron ha tomado demasiados riesgos y la batería llega a 0 antes de llegar a la meta el dron ha "perdido"
        if battery == 0:
            print("Game over: You have run out of battery :(")
            break
        
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()