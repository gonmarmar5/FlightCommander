'''
robotica.py

Provides the communication between CoppeliaSim robotics simulator and
external Python applications via the ZeroMQ remote API.

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

import random
import numpy as np
import time
import math

from zmqRemoteApi import RemoteAPIClient


class Coppelia():

    def __init__(self):
        print('*** connecting to coppeliasim')
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

    def start_simulation(self):
        # print('*** saving environment')
        self.default_idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.startSimulation()

    def stop_simulation(self):
        # print('*** stopping simulation')
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)
        # print('*** restoring environment')
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.default_idle_fps)
        print('*** done')

    def is_running(self):
        return self.sim.getSimulationState() != self.sim.simulation_stopped
    
class QuadCopter():
    def __init__(self, sim, robot_id):
        self.sim = sim
        self.target = sim.getObject(f'/{robot_id}/target')
        self.base = sim.getObject(f'/{robot_id}/base')

        self.lidar = []
        self.num_lidar = 6
        for i in range(self.num_lidar):
            self.lidar.append(self.sim.getObject(f'/{robot_id}/Cuboid/fastHokuyo[{i}]'))

        self.objects = []
        self.number_objects = 50
        for i in range(self.number_objects):
            self.objects.append(self.sim.getObject(f'/Obstacle[{i}]'))


        #self.angles = [((((-i + (len(distances) / 2 - 1)) / (len(distances) / 2 - 1)) * 120) / 180) * np.pi for i in range(len(distances))]

    def getposition_target(self):
        return self.sim.getObjectPosition(self.target, self.sim.handle_world)

    def getposition_drone(self):
        return self.sim.getObjectPosition(self.base, self.sim.handle_world), self.sim.getObjectOrientation(self.base, self.sim.handle_world)

    def setposition_target(self, new_position):
        print(list(new_position))
        pos = self.sim.getObjectPosition(self.target, self.sim.handle_world)
        self.sim.setObjectPosition(self.target, -1, new_position, self.sim.handle_world)

    def activate_objects(self, objects_to_init):
        for i in objects_to_init:
            self.sim.setObjectInt32Param(self.objects[i], self.sim.objintparam_collection_self_collision_indicator, True)

    def get_lidar(self):
        lidar_data = []

        for lidar in self.lidar:
            data = self.sim.getStringSignal(lidar)
            distances = self.sim.unpackFloatTable(data)
            angles = [((((-i + (len(distances) / 2 - 1)) / (len(distances) / 2 - 1)) * 120) / 180) * np.pi for i in range(len(distances))]
            lidar_data.append(distances, angles)

        return lidar_data
