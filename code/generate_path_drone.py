import math
import networkx as nx
from queue import PriorityQueue
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import pickle


class generate_path_planning():
    def __init__(self, path, algorithm, heuristic, start, goal):
        # Convertimos el negro en blanco y el blanco en negro
        image = np.array(Image.open(path))
        rgb_image = image[:, :, :3]
        image = np.mean(rgb_image, axis=2).astype(np.uint8)
        image = np.round((255 - image)/255)
        
        # creamos el graph
        graph = self.create_graph(image)
        bounds = np.array([[0, self.rows], [0, self.cols]])
        heuristics = ["manhattan", "diagonal", "octile", "euclidean"]

        # llamamos a cada algoritmo de búsqueda según lo que haya determinado el usuario
        identifier = 0
        if algorithm == 1:
            path, visited = self.depth_first_search(graph, start, goal)
        elif algorithm == 2:
            path, visited = self.breadth_first_search(graph, start, goal)
        elif algorithm == 3:
            path, visited = self.dijkstra(graph, start, goal)
        elif algorithm == 4:
            if heuristic != 5 and heuristic != 6:
                path, visited = self.greedy_best_first_search(graph, start, goal, heuristics[heuristic - 1])
            elif heuristic == 5:
                for heuristic in heuristics:
                    print("The heuristic being used is", heuristic)
                    path, visited = self.greedy_best_first_search(graph, start, goal, heuristic)

                    if path != None:
                        self.plot_path_and_visited_node(image, path, visited)
                        print("The size of the path is", len(path))
                        print("There has been visited", len(visited), "nodes")
                        identifier = 1
                    else:
                        print("Path not found")
        elif algorithm == 5:
            if heuristic != 5 and heuristic != 6:
                self.path, visited = self.astar(graph, start, goal, heuristics[heuristic - 1])
            elif heuristic == 5:
                for heuristic in heuristics:
                    print("The heuristic being used is", heuristic)
                    path, visited = self.astar(graph, start, goal, heuristic)

                    if path != None:
                        self.plot_path_and_visited_node(image, path, visited)
                        print("The size of the path is", len(path))
                        print("There has been visited", len(visited), "nodes")
                        identifier = 1
                    else:
                        print("Path not found")
        elif algorithm == 6:
            path, visited = self.wavefront_algorithm(image, graph, start, goal)
        elif algorithm == 7:
            path, visited = self.prm_algorithm(bounds, 1000, 10, start, goal, graph, "euclidean")
            distance = 0
            for i in range(len(path)):
                try:
                    distance += np.sqrt((path[i][0] - path[i + 1][0])**2 + (path[i][1] - path[i + 1][1])**2) # estimate the distance of the path
                except:
                    pass
        elif algorithm == 8:
            path, visited = self.rrt_algorithm(bounds, 3000, start, goal, graph, "euclidean")
            distance = 0
            for i in range(len(path)):
                try:
                    distance += np.sqrt((path[i][0] - path[i + 1][0])**2 + (path[i][1] - path[i + 1][1])**2) # estimate the distance of the path
                except:
                    pass
        elif algorithm == 9:
            identifier = 1
            print("depth first search: ")
            path, visited = self.depth_first_search(graph, start, goal)
            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", len(path))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

            print("breadth first search: ")
            path, visited = self.breadth_first_search(graph, start, goal)
            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", len(path))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

            print("dijkstra first search: ")
            path, visited = self.dijkstra(graph, start, goal)
            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", len(path))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

            print("greedy best first search: ")
            if heuristic != 5 and heuristic != 6:
                path, visited = self.greedy_best_first_search(graph, start, goal, heuristics[heuristic - 1])
                if path != None:
                    self.plot_path_and_visited_node(image, path, visited)
                    print("The size of the path is", len(path))
                    print("There has been visited", len(visited), "nodes")
                else:
                    print("Path not found")
            elif heuristic == 5:
                for heuristic_ in heuristics:
                    print("The heuristic being used is", heuristic_)
                    path, visited = self.greedy_best_first_search(graph, start, goal, heuristic_)

                    if path != None:
                        self.plot_path_and_visited_node(image, path, visited)
                        print("The size of the path is", len(path))
                        print("There has been visited", len(visited), "nodes")
                    else:
                        print("Path not found")
            
            print("a star: ")
            if heuristic != 5 and heuristic != 6:
                self.path, visited = self.astar(graph, start, goal, heuristics[heuristic - 1])
                if self.path != None:
                    self.plot_path_and_visited_node(image, path, visited)
                    print("The size of the path is", len(path))
                    print("There has been visited", len(visited), "nodes")
                else:
                    print("Path not found")
            elif heuristic == 5:
                for heuristic_ in heuristics:
                    print("The heuristic being used is", heuristic_)
                    path, visited = self.astar(graph, start, goal, heuristic_)

                    if path != None:
                        self.plot_path_and_visited_node(image, path, visited)
                        print("The size of the path is", len(path))
                        print("There has been visited", len(visited), "nodes")
                    else:
                        print("Path not found")

            print("wavefront algorithm: ")
            path, visited = self.wavefront_algorithm(image, graph, start, goal)
            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", len(path))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

            print("prm algorithm: ")
            path, visited = self.prm_algorithm(bounds, 1000, 10, start, goal, graph, "euclidean")

            distance = 0
            for i in range(len(path)):
                try:
                    distance += np.sqrt((path[i][0] - path[i + 1][0])**2 + (path[i][1] - path[i + 1][1])**2) # estimate the distance of the path
                except:
                    pass

            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", round(distance))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

            print("rtt algorithm: ")
            path, visited = self.rrt_algorithm(bounds, 5000, start, goal, graph, "euclidean")

            distance = 0
            for i in range(len(path)):
                try:
                    distance += np.sqrt((path[i][0] - path[i + 1][0])**2 + (path[i][1] - path[i + 1][1])**2) # estimate the distance of the path
                except:
                    pass

            if path != None:
                self.plot_path_and_visited_node(image, path, visited)
                print("The size of the path is", round(distance))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

        if identifier == 0:
            # printeamos el graph y el camino
            if self.path != None:
                self.plot_path_and_visited_node(image, self.path, visited)
                try:
                    print("The size of the path is", round(distance))
                except:
                    print("The size of the path is", len(self.path))
                print("There has been visited", len(visited), "nodes")
            else:
                print("Path not found")

        # guardamos el graph y el camino
        #with open('D:/graph.pkl', 'wb') as file:
        #    pickle.dump(graph, file)

        #with open('D:/path_high.pkl', 'wb') as file:
        #   pickle.dump(path, file)
                
    def return_path(self):
        return self.path

    def create_graph(self, image):
        graph = nx.Graph()
        self.rows, self.cols = image.shape

        # Cada píxel igual a 1 es un node (si es igual a 0 es un obstáculo)
        for i in range(self.rows):
            for j in range(self.cols):
                if image[i, j] == 1:
                    graph.add_node((i, j))

        # Se crea un edge entre nodes que están a una distancia 1
        for i in range(self.rows):
            for j in range(self.cols):
                if image[i, j]:
                    neighbors = [
                        (i - 1, j),
                        (i + 1, j),
                        (i, j - 1),
                        (i, j + 1),
                        (i + 1, j + 1),
                        (i - 1, j - 1),
                        (i + 1, j - 1),
                        (i - 1, j + 1)
                    ]
                    for neighbor in neighbors:
                        if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols and image[neighbor] == 1:
                            graph.add_edge((i, j), neighbor)

        print(len(graph.nodes))
        # borramos todos los nodos a una distancia 5 de un obstáculo para simular el tamaño del dron
        nodes = []
        for i in list(graph.nodes):
            try:
                for j in range(1, 11):
                    neighbors = [
                        (i[0] + j, i[1]),
                        (i[0] - j, i[1]),
                        (i[0], i[1] + j),
                        (i[0], i[1] - j),
                        (i[0] + j, i[1] + j),
                        (i[0] + j, i[1] - j),
                        (i[0] - j, i[1] + j),
                        (i[0] - j, i[1] - j)
                    ]

                    if any(not graph.has_node(neighbor) for neighbor in neighbors):
                        if i not in nodes:
                            nodes.append(i)
            except:
                pass

        for i in nodes:
            graph.remove_node(i)

        print(len(graph.nodes))

        return graph
    
    def heuristic(self, node, goal, distance_metric='euclidean'):
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
    
    def reconstruct_path(self, came_from, start, goal):
        current_node = goal
        path = []

        while current_node != start:
            path.append(current_node)
            current_node = came_from[current_node]

        path.append(start)
        return path[::-1]
    
    def get_neighbors_wavefront(self, pos):
        x, y = pos
        neighbors = []
        neighbors.append((x - 1, y))
        neighbors.append((x + 1, y))
        neighbors.append((x, y - 1))
        neighbors.append((x, y + 1))
        neighbors.append((x + 1, y + 1))
        neighbors.append((x - 1, y + 1))
        neighbors.append((x - 1, y - 1))
        neighbors.append((x + 1, y - 1))

        return neighbors # Devuelve todos los posibles vecinos

    def reconstruct_path_wavefront(self, wavefront_grid, start):
        x, y = start
        path = [(x, y)]
        while wavefront_grid[x, y] != 0:
            neighbors = self.get_neighbors_wavefront((x, y))
            for nx, ny in neighbors:
                if wavefront_grid[nx, ny] == wavefront_grid[x, y] - 1:
                    path.append((nx, ny))
                    x, y = nx, ny
                    break
        return path[::-1]
    
    def generate_random_point(self, bounds):
        return np.random.uniform(bounds[:, 0], bounds[:, 1]) # Genera un punto random dentro de los bordes

    def is_collision_free(self, point1, point2, occupancy_grid):
        # Se asegura de que el camino entre el punto 1 y el punto 2 exista y no esté tapados por obstáculos
        num_steps = 100 
        path = np.linspace(point1, point2, num_steps)

        for p in path:
            p_rounded = np.round(p).astype(int)
            try:
                occupancy_grid[p_rounded[0], p_rounded[1]]
            except:
                return False  # Si hay un obstáculo devuelve falso

        return True  # Y sino devuelve verdadero
    
    def find_nearest_node(self, tree, point):
        distances = np.linalg.norm(np.array(list(tree.keys())) - np.array(point), axis=1)
        nearest_node = list(tree.keys())[np.argmin(distances)]
        return nearest_node
    
    def depth_first_search(self, graph, start, goal):
        visited = set()
        stack = [(start, [start])]

        while stack:
            current_node, current_path = stack.pop()

            if current_node in visited: 
                continue

            visited.add(current_node) # Guarda los nodos que ha visitado

            if current_node == goal:
                return current_path, visited # Devuelve el camino

            for neighbor in graph.neighbors(current_node): # Itera entre todos los vecinos dando prioridad a los hijos antes que a los hermanos
                if neighbor not in visited:
                    new_path = current_path + [neighbor]
                    stack.append((neighbor, new_path))

        return None, visited  # No hay camino entre start y goal
    
    def breadth_first_search(self, graph, start, goal):
        visited = set()
        queue = [(start, [start])]

        while queue:
            current_node, current_path = queue.pop(0)

            if current_node in visited:
                continue

            visited.add(current_node) # Guarda los nodos que ha visitado

            if current_node == goal:
                return current_path, visited # Devuelve el camino

            for neighbor in graph.neighbors(current_node): # Itera entre todos los vecinos dando prioridad a los hermanos antes que a los hijos
                if neighbor not in visited:
                    new_path = current_path + [neighbor]
                    queue.append((neighbor, new_path))

        return None, visited  # No hay camino entre start y goal
    
    def dijkstra(self, graph, start, goal):
        visited = set()
        distance = {node: float('inf') for node in graph.nodes}
        distance[start] = 0
        came_from = {node: None for node in graph.nodes}

        priority_queue = [(0, start)]

        while priority_queue:
            current_dist, current_node = min(priority_queue)
            priority_queue.remove((current_dist, current_node))

            visited.add(current_node) # Guarda los nodos que ha visitado

            if current_node == goal:
                path = [goal]
                while came_from[goal] is not None:
                    path.insert(0, came_from[goal])
                    goal = came_from[goal]
                return path, visited # Devuelve el camino

            for neighbor, weight in graph[current_node].items():
                try:
                    new_dist = distance[current_node] + graph[current_node][neighbor]["weight"] # Itera dando prioridad primero a los nodos que esten más cerca del starting point
                except:
                    new_dist = distance[current_node] + 1
                if new_dist < distance[neighbor]:
                    distance[neighbor] = new_dist
                    came_from[neighbor] = current_node
                    priority_queue.append((new_dist, neighbor))

        return None, visited # No hay camino entre start y goal

    def greedy_best_first_search(self, graph, start, goal, heuristic):
        visited = set()
        priority_queue = [(self.heuristic(start, goal, heuristic), start, [start])]

        while priority_queue:
            _, current_node, current_path = min(priority_queue)
            priority_queue.remove((self.heuristic(current_node, goal, heuristic), current_node, current_path))

            if current_node in visited:
                continue

            visited.add(current_node) # Guarda los nodos que ha visitado

            if current_node == goal:
                return current_path, visited # Devuelve el camino

            for neighbor in graph.neighbors(current_node): # Itera dando prioridad primero a los nodos que minimizan una heurística la cual valora como de cerca queda la meta
                if neighbor not in visited:
                    new_path = current_path + [neighbor]
                    priority_queue.append((self.heuristic(neighbor, goal, heuristic), neighbor, new_path))

        return None, visited  # No hay camino entre start y goal

    def astar(self, graph, start, goal, heuristic):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        cost_so_far = {start: 0}
        visited_nodes = set()

        while not open_set.empty():
            current_cost, current_node = open_set.get()
            visited_nodes.add(current_node) # Guarda los nodos que ha visitado

            if current_node == goal:
                path = self.reconstruct_path(came_from, start, goal)
                return path, visited_nodes # Devuelve el camino

            for next_node in list(graph.neighbors(current_node)):
                try:
                    new_cost = cost_so_far[current_node] + graph[current_node][next_node]["weight"]
                except:
                    new_cost = cost_so_far[current_node] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]: # Itera dando prioridad primero a los nodos que minimizan una heurística la cual valora como de cerca queda la meta y que tienen un coste menor desde la posición inicial
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal, heuristic)
                    open_set.put((priority, next_node))
                    came_from[next_node] = current_node

        return None, visited_nodes  # No hay camino entre start y goal
    
    def wavefront_algorithm(self, image, graph, start, goal):
        rows, cols = image.shape
        wavefront_grid = -1 * np.ones_like(image)  # Inicias todos los nodes en -1
        visited_nodes = set()

        goal_x, goal_y = goal
        wavefront_grid[goal_x, goal_y] = 0  # Menos la meta que es donde quieres empezar a reconstruir las distancias
        visited_nodes.add((goal_x, goal_y))

        wave_value = 1

        while wavefront_grid[start[0], start[1]] == -1:
            for x in range(rows):
                for y in range(cols):
                    if wavefront_grid[x, y] == wave_value - 1:
                        neighbors = graph.neighbors((x,y))
                        for nx, ny in neighbors:
                            if wavefront_grid[nx, ny] == -1:  # Empiezas por la meta y vas dando distancias a los vecinos. La siguiente iteración le daras distancias a los vecinos de los vecinos de la meta. Sigues hasta haber creado un camino desde la meta hasta el principio
                                wavefront_grid[nx, ny] = wave_value
                                visited_nodes.add((nx, ny))

            wave_value += 1

        path = self.reconstruct_path_wavefront(wavefront_grid, start) # Finalmente reconstruyes el camino
        return path, visited_nodes
    
    def prm_algorithm(self, bounds, num_samples, num_neighbors, start, goal, occupancy_grid, heuristic):
        new_graph = nx.Graph()
        # genera puntos al azar
        sampled_points = [self.generate_random_point(bounds) for _ in range(num_samples)]
        sampled_points = np.round(sampled_points)

        # se asegura que esten en espacios disponibles y si lo está añade esos puntos como nodes
        valid_points = []
        for point in sampled_points:
            if self.is_collision_free(point, point, occupancy_grid) == True:
                new_graph.add_node(tuple(point))
                valid_points.append(tuple(point))

        # añade la meta y el principio del camino ya que al final son los puntos que queremos conectar
        valid_points.append(tuple(start))
        valid_points.append(tuple(goal))

        new_graph.add_node(tuple(start))
        new_graph.add_node(tuple(goal))

        for i, point in enumerate(valid_points):
            # para cada punto ordena los puntos segun como de cerca esten y para el número de vecinos seleccionado, mira si hay un camino disponible entre esos puntos y si lo hay, lo añade como vecino. De esta forma está creando de forma probabilistica un grafo mucho más pequeño del cual sacar un camino entre la meta y el starting point
            distances = np.linalg.norm(np.array(valid_points) - np.array(point), axis=1)
            sorted_indices = np.argsort(distances)
            
            for j in sorted_indices[1:num_neighbors + 1]:
                if distances[j] > 0 and self.is_collision_free(point, valid_points[j], occupancy_grid):
                    
                    new_graph.add_edge(point, valid_points[j])

        path, visited_2 = self.astar(new_graph, start, goal, heuristic)

        return path, set(valid_points) # finalmente crea el mejor camino a partir del nuevo grafo que hemos creado con los puntos seleccionados de forma random
    
    def rrt_algorithm(self, bounds, num_samples, start, goal, graph, euclidean):
        tree = {start: None}
        new_graph = nx.Graph()
        new_graph.add_node(start)
        goal_bias = 0.1

        for _ in range(num_samples):
            if np.random.rand() < goal_bias:
                random_point = goal  # Añadimos una probabilidad de que coja directamente la meta como punto para hacer más probable que se acerque a esta
            else:
                random_point = self.generate_random_point(bounds) # Sino, generamos un punto al azar

            random_point = np.round(random_point)

            nearest_node = self.find_nearest_node(tree, random_point)

            # Calculamos un nodo final en función a la distancia con el nodo que más cerca le queda y donde se ha generado el random point
            new_node = np.clip(nearest_node + 0.4 * (random_point - nearest_node), bounds[:, 0], bounds[:, 1])

            new_node = np.round(new_node)

            if (new_node[0], new_node[1]) not in tree:

                # Nos aseguramos que existe un camino entre los dos nodos
                if not self.is_collision_free(new_node, nearest_node, graph):
                    identifier = 0
                    for nearest_node in tree:
                        if self.is_collision_free(nearest_node, new_node, graph): # Si no existe buscamos otro nodo para el que si que exista el camino con el nuevo nodo
                            identifier = 1
                            break
                    if identifier == 0:
                        continue
                
                # Si existe un camino añadimos el nuevo nodo y los conectamos
                tree[tuple(new_node)] = nearest_node
                new_graph.add_node(tuple(new_node))
                new_graph.add_edge(tuple(new_node), nearest_node)

                # Además si existe un camino con la meta también los conectamos, y terminamos, ya que empezando solo con start hemos podido encontrar una serie de puntos que van desde el principio hasta la meta sin chocar
                if self.is_collision_free(new_node, goal, graph):
                    tree[goal] = tuple(new_node)
                    new_graph.add_node(goal)
                    new_graph.add_edge(goal, tuple(new_node))
                    break

        path, visited_2 = self.astar(new_graph, start, goal, euclidean)

        return path, set(tree.keys()) # Finalmente reconstruimos el camino usando a star

    
    def plot_path_and_visited_node(self, image, path, visited_nodes):
        plt.imshow(image, cmap='gray')

        # Saca las coordenadas x,y del camino
        x, y = zip(*path)

        # Dibuja el camino
        plt.plot(y, x, marker='.', color='red', markersize=5, label='Path')

        # Saca las coordenadas x,y de los nodos visitados
        x_visited, y_visited = zip(*visited_nodes)

        # Dibuja los nodos visitados
        #plt.scatter(y_visited, x_visited, color='blue', s=2, label='Visited Nodes')

        plt.legend()
        plt.show()

if __name__ == '__main__':
    paths = ["D:/processed_image_map.png"]

    # Le preguntamos al usuario que algoritmo y con que heurística quiere ejecutarlo
    print('Which algorithm do you want to apply?: 1. depth first search, 2. breadth first search, 3. dijkstra, 4. greedy best first search, 5. a star, 6. wavefront algorithm, 7. prm algorithm, 8.rrt algorithm, 9. all of them')
    algorithm = int(input())

    if algorithm == 4 or algorithm == 5 or algorithm == 9:
        print('Which heuristic do you want to apply?: 1. manhattan, 2. diagonal, 3. octile, 4. euclidean, 5. all of them')
        heuristic = int(input())
    else:
        heuristic = 6

    # como se ve el camino rápido se puede sacar de golpe ya que es el camino óptimo y por lo tanto el que encuentra el A*. Los otros dos necesitan ser sacados por trozos y juntarse al final ya que no son los óptimos si todos los caminos tienen los mismos pesos en todas las aristas
    for path in paths:
        # sacamos el camino rápido
        generate_path_planning(path, algorithm, heuristic, (49,119), (288,211))
        # sacamos el camino medio

        #plan = generate_path_planning(path, algorithm, heuristic, (49,119), (277,111))
        #path_2 = plan.return_path()
        #plan = generate_path_planning(path, algorithm, heuristic, (277,111), (288,211))
        #path_3 = plan.return_path()
        # sacamos el camino lento
        #plan = generate_path_planning(path, algorithm, heuristic, (49,119), (65,323))
        #path_2 = plan.return_path()
        #plan = generate_path_planning(path, algorithm, heuristic, (65,323), (288,211))
        #path_3 = plan.return_path()
#
        #path_2 += path_3

        # guardamos el camino
        #with open('D:/path_low.pkl', 'wb') as file:
        #   pickle.dump(path_2, file)