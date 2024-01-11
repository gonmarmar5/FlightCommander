import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

EXTEND_AREA = 0

def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


def calc_grid_map_config(ox, oy, xy_resolution, existing_min_x=None, existing_max_x=None, existing_min_y=None, existing_max_y=None):
    """
    Calculates the size, and the maximum distances according to the
    measurement center
    """
    # Alterado para no dividir de momento el min por la resolución y para coger el mínimo entre el mínimo del nuevo y el antiguo mapa
    min_x = (min(ox) - EXTEND_AREA)
    min_y = (min(oy) - EXTEND_AREA)
    max_x = (max(ox) + EXTEND_AREA)
    max_y = (max(oy) + EXTEND_AREA)
    if existing_min_x != None:
        min_x = min(min_x, existing_min_x)
        min_y = min(min_y, existing_min_y)
        max_x = max(max_x, existing_max_x)
        max_y = max(max_y, existing_max_y)
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw


def init_flood_fill(center_point, obstacle_points, xy_points, min_coord, xy_resolution):
    """
    center_point: center point
    obstacle_points: detected obstacles points (x,y)
    xy_points: (x,y) point pairs
    """
    center_x, center_y = center_point
    prev_ix, prev_iy = center_x - 1, center_y
    ox, oy = obstacle_points
    xw, yw = xy_points
    occupancy_map = (np.ones((xw, yw))) * 0.5

    def within_bounds(x, y):
        return 0 <= x < xw and 0 <= y < yw

    for (x, y) in zip(ox, oy):
        # x coordinate of the occupied area
        ix = int(round((x - min_coord[0]) / xy_resolution))
        # y coordinate of the occupied area
        iy = int(round((y - min_coord[1]) / xy_resolution))
        free_area = bresenham((prev_ix, prev_iy), (ix, iy))
        for fa in free_area:
            if within_bounds(fa[0], fa[1]):
                occupancy_map[fa[0]][fa[1]] = 0  # free area 0.0
        prev_ix = ix
        prev_iy = iy

    return occupancy_map



def flood_fill(center_point, occupancy_map):
    """
    center_point: starting point (x,y) of fill
    occupancy_map: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method
    sx, sy = occupancy_map.shape
    fringe = deque()
    fringe.appendleft(center_point)
    
    def within_bounds(x, y):
        return 0 <= x < sx and 0 <= y < sy

    while fringe:
        n = fringe.pop()
        nx, ny = n

        # West
        if nx > 0 and within_bounds(nx - 1, ny) and occupancy_map[nx - 1, ny] == 0.5:
            occupancy_map[nx - 1, ny] = 0.0
            fringe.appendleft((nx - 1, ny))

        # East
        if nx < sx - 1 and within_bounds(nx + 1, ny) and occupancy_map[nx + 1, ny] == 0.5:
            occupancy_map[nx + 1, ny] = 0.0
            fringe.appendleft((nx + 1, ny))

        # North
        if ny > 0 and within_bounds(nx, ny - 1) and occupancy_map[nx, ny - 1] == 0.5:
            occupancy_map[nx, ny - 1] = 0.0
            fringe.appendleft((nx, ny - 1))

        # South
        if ny < sy - 1 and within_bounds(nx, ny + 1) and occupancy_map[nx, ny + 1] == 0.5:
            occupancy_map[nx, ny + 1] = 0.0
            fringe.appendleft((nx, ny + 1))



def generate_ray_casting_grid_map(ox, oy, xy_resolution, breshen=True, position=None, existing_occupancy_map=None, existing_min_x=None, existing_max_x=None, existing_min_y=None, existing_max_y=None):
    """
    The breshen boolean tells if it's computed with bresenham ray casting
    (True) or with flood fill (False)
    """
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(
        ox, oy, xy_resolution, existing_min_x, existing_max_x, existing_min_y, existing_max_y)
    # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
    
    occupancy_map = np.ones((x_w, y_w)) / 2

    # Alterado para recalcular las coordenadas del antiguo mapa en función al nuevo mínimo
    first_i = None
    first_j = None
    if existing_min_x != None:
        for i in range(int((min_x - min_x)/xy_resolution), int((round(max_x - min_x)/xy_resolution))):
            if i >= (existing_min_x - min_x)/xy_resolution and i < (existing_max_x - min_x)/xy_resolution:
                for j in range(int((min_y - min_y)/xy_resolution), int(round((max_y - min_y)/xy_resolution))):
                    if j >= (existing_min_y - min_y)/xy_resolution and j < (existing_max_y - min_y)/xy_resolution:
                        if first_i == None:
                            first_i = i
                        
                        if first_j == None:
                            first_j = j

                        try:
                            occupancy_map[i][j] = existing_occupancy_map[i - first_i][j - first_j]
                        except:
                            pass

    center_x = int(
        round(-min_x / xy_resolution))  # center x coordinate of the grid map
    center_y = int(
        round(-min_y / xy_resolution))  # center y coordinate of the grid map
    
    # occupancy grid computed with bresenham ray casting
    if breshen:
        counter = 0
        
        for (x, y) in zip(ox, oy):
            # Alterada para recalcular las coordenadas de este mapa en función al nuevo mínimo
            ix = int(round((x + (min(ox) - min_x) - min(ox)) / xy_resolution))
            iy = int(round((y + (min(oy) - min_y) - min(oy)) / xy_resolution))
            
            # Antes usaban min(ox) porque el mínimo era el punto donde (cos(ang)) = 0 y por lo tanto ox = posición del robot, así que como nosotros no usamos las distancias = 5, podemos perder este mínimo así que lo que hacemos es traer la posición y transformarla de la misma manera que el resto de puntos ox
            center_x_2 = int(round((position[0] + (min(ox) - min_x) - min(ox)) / xy_resolution))
            center_y_2 = int(round((position[1] + (min(oy) - min_y) - min(oy)) / xy_resolution))

            # Después ya calculamos bresenham como siempre
            laser_beams = bresenham((center_x_2, center_y_2), (ix, iy))  # line form the lidar to the occupied point
            for laser_beam in laser_beams:
                try:
                    if occupancy_map[laser_beam[0]][laser_beam[1]] == 0.5:
                        occupancy_map[laser_beam[0]][laser_beam[1]] = 0.0  # free area 0.0
                except:
                    pass

            try:
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
            except:
                pass
            try:
                occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            except:
                pass
            try:
                occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            except:
                pass
            try:
                occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
            except:
                pass
            counter += 1

    # occupancy grid computed with flood fill
    else:
        #occupancy_map = init_flood_fill((center_x, center_y), (ox, oy),
        #                                (x_w, y_w),
        #                                (min_x, min_y), xy_resolution)
        flood_fill((center_x, center_y), occupancy_map)
        occupancy_map = np.array(occupancy_map, dtype=float)
        for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))
            try:
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
            except:
                pass
            try:
                occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            except:
                pass
            try:
                occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            except:
                pass
            try:
                occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
            except:
                pass
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution


def generate_map(ang, dist, position, orientation, existing_occupancy_map=None, existing_min_x=None, existing_max_x=None, existing_min_y=None, existing_max_y=None):
    xy_resolution = 0.01  # x-y grid resolution
    
    robot_x, robot_y, robot_z = position
    alpha, beta, gamma = orientation
    
    ox = []
    oy = []
    for i in range(0, len(ang)):
        if dist[i] != 5: # si la distancia es menor de 5 (si es 5 es que el lidar no da para más no que haya un objeto)
            ox.append(np.cos(-ang[i] + gamma) * dist[i] + robot_x) # se recalcula la posición de los objetos obtenida a partir del lidar en función a la posición y orientación del robot
            oy.append(np.sin(-ang[i] + gamma) * dist[i] + robot_y)

    try:
        if existing_occupancy_map is None:
            # Si no existe ningún mapa se crea uno nuevo
            occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
                generate_ray_casting_grid_map(ox, oy, xy_resolution, True, position)
        else:
            # Sino se actualiza el antiguo en función al nuevo
            occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = generate_ray_casting_grid_map(ox, oy, xy_resolution, True, position, existing_occupancy_map, existing_min_x, existing_max_x, existing_min_y, existing_max_y)
            
        return occupancy_map, min_x, max_x, min_y, max_y
    except:
        return existing_occupancy_map, existing_min_x, existing_max_x, existing_min_y, existing_max_y
    

def print_map(occupancy_map):
    xy_res = np.array(occupancy_map).shape
    plt.figure(1, figsize=(10, 4))
    plt.imshow(occupancy_map, cmap="viridis")
    # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
    plt.colorbar()
    plt.grid(True)
    plt.show()
