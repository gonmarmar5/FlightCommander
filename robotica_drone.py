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
        print('*** getting handles', robot_id)

        # Atributos de las particulas debajo del dron
        self.particlesAreVisible = True
        self.simulateParticles = True
        self.fakeShadow = True

        self.particleCountPerSecond = 430
        self.particleSize = 0.005
        self.particleDensity = 8500
        self.particleScatteringAngle = 30
        self.particleLifeTime = 0.5
        self.maxParticleCount = 50
        self.velocity = [0.1, 0, 0]

        self.propellerHandles = []
        self.jointHandles = []
        self.particleObjects = [-1, -1, -1, -1]

        self.ttype = (sim.particle_roughspheres + sim.particle_cyclic +
                sim.particle_respondable1to4 + sim.particle_respondable5to8 +
                sim.particle_ignoresgravity)
        
        if not self.particlesAreVisible:
            self.ttype += self.sim.particle_invisible

        for i in range(4):
            self.propellerHandles.append(sim.getObject(f'/{robot_id}/propeller[{i}]/respondable'))
            self.jointHandles.append(sim.getObject(f'/{robot_id}/propeller[{i}]/joint'))

            # Crea particulas debajo de los motores:
            if self.simulateParticles:
                self.particleObjects[i] = self.sim.addParticleObject(
                    self.ttype, self.particleSize, self.particleDensity, [2, 1, 0.2, 3, 0.4],
                    self.particleLifeTime, self.maxParticleCount, [0.3, 0.7, 1]
                )

        if self.fakeShadow:
            self.shadowCont = self.sim.addDrawingObject(
                self.sim.drawing_discpts + self.sim.drawing_cyclic +
                self.sim.drawing_25percenttransparency + self.sim.drawing_50percenttransparency +
                self.sim.drawing_itemsizes, 0.2, 0, -1, 1
            )

        # Conecta con el dron segun su robot_id
        self.d = sim.getObject(f'/{robot_id}/base')
        self.heli = sim.getObject(f'/{robot_id}')
        
        self.pParam = 2
        self.iParam = 0
        self.dParam = 0
        self.vParam = -2
        self.cumul = 0
        self.lastE = 0
        self.pAlphaE = 0
        self.pBetaE = 0
        self.psp2 = 0
        self.psp1 = 0
        self.prevEuler = 0

    def cleanup(self):
        '''
        Elimina los objetos creados en la simulacion
        '''

        self.sim.removeDrawingObject(self.shadowCont)
        for i in range(len(self.particleObjects)):
            self.sim.removeParticleObject(self.particleObjects[i])

    def actuation(self, speed):
        '''
        Actua sobre el dron para que se mueva en la direccion indicada
        '''
        # Obtiene las coordenadas segun el sistema de referencia del mundo (x,y,z)
        pos = self.sim.getObjectPosition(self.d, self.sim.handle_world)
        
        # Crea un objeto que proyecta una sombra debajo del dron
        if self.fakeShadow:
            itemData = [pos[0], pos[1], 0.002, 0, 0, 0, 1, 0.2]
            self.sim.addDrawingObjectItem(self.shadowCont, itemData)

        ## Control vertical:
        # Target position: La posicion actual mas la velocidad (x,y,z)
        targetPos = np.array(pos) + np.array(speed)
        
        # Velocidad lineal y angular del dron
        l = self.sim.getVelocity(self.heli)
        e = (targetPos[2] - pos[2])

        self.cumul = self.cumul + e
        pv = self.pParam * e
        # Fuerza de empuje aplicada al dron
        thrust = 5.45 + pv + self.iParam * self.cumul + self.dParam * (e - self.lastE) + l[0][2] * self.vParam
        self.lastE = e

        ## Horizontal control:

        # Obtiene la matriz de transformación del dron en relación con el mundo
        m = self.sim.getObjectMatrix(self.d, self.sim.handle_world) # Array of 12 values [Vx0 Vy0 Vz0 P0 Vx1 Vy1 Vz1 P1 Vx2 Vy2 Vz2 P2]
        
        # Define y transforma el vector de dirección horizontal x del dron en el sistema de referencia del mundo
        vx = [1, 0, 0]
        vx = self.sim.multiplyVector(m, vx)

        # Define y transforma el vector de dirección horizontal y del dron en el sistema de referencia del mundo
        vy = [0, 1, 0]
        vy = self.sim.multiplyVector(m, vy)

        # Calcula el error en la inclinación alpha del dron, vy[2] es la componente z del vector vy
        alphaE = (vy[2] - m[11])
        alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - self.pAlphaE)
        self.pAlphaE = alphaE

        # Calcula el error en la inclinación beta del dron, vx[2] es la componente z del vector vx
        betaE = (vx[2] - m[11])
        betaCorr = -0.25 * betaE - 2.1 * (betaE - self.pBetaE)
        self.pBetaE = betaE
        
        alphaCorr = alphaCorr + speed[1] * 0.005 + 1 * (speed[1] - self.psp2)
        self.psp2 = speed[1]

        betaCorr = betaCorr - speed[0] * 0.005 - 1 * (speed[0] - self.psp1)
        self.psp1 = speed[0]

        ## Control Rotacional:
        euler=self.sim.getObjectOrientation(self.d,self.sim.handle_world)
        rotCorr=euler[2]*0.1+2*(euler[2]-self.prevEuler)
        self.prevEuler=euler[2]

        # Decide las velocidades de los motores :
        self.handle_propeller(1, thrust * (1 - alphaCorr + betaCorr + rotCorr))
        self.handle_propeller(2, thrust * (1 - alphaCorr - betaCorr - rotCorr))
        self.handle_propeller(3, thrust * (1 + alphaCorr - betaCorr + rotCorr))
        self.handle_propeller(4, thrust * (1 + alphaCorr + betaCorr - rotCorr))

    def handle_propeller(self, index, particle_velocity):
        '''
        Actua sobre el motor indicado para que se mueva a la velocidad indicada
        '''
        propellerRespondable = self.propellerHandles[index - 1]
        propellerJoint = self.jointHandles[index - 1]
        propeller = self.sim.getObjectParent(propellerRespondable)

        notFullParticles = 0
        t = self.sim.getSimulationTime()
        self.sim.setJointPosition(propellerJoint, t * 10)
        ts = self.sim.getSimulationTimeStep()

        m = self.sim.getObjectMatrix(propeller, self.sim.handle_world)

        requiredParticleCnt = self.particleCountPerSecond * ts + notFullParticles
        notFullParticles = requiredParticleCnt % 1
        requiredParticleCnt = math.floor(requiredParticleCnt)

        # Apply a reactive force onto the body:
        totalExertedForce = (requiredParticleCnt * self.particleDensity * particle_velocity *
                            math.pi * self.particleSize * self.particleSize *
                            self.particleSize) / (6 * ts)
        force = [0, 0, totalExertedForce]
        m[3] = 0
        m[7] = 0
        m[11] = 0
        force = self.sim.multiplyVector(m, force)
        rotDir = 1 - np.mod(index, 2) * 2
        torque = [0, 0, rotDir * 0.002 * particle_velocity]
        torque = self.sim.multiplyVector(m, torque)
        self.sim.addForceAndTorque(propellerRespondable, force, torque)

def main(args=None):
    coppelia = Coppelia()
    robot = QuadCopter(coppelia.sim, 'Quadcopter')
    speed = [0,0,0]
    robot.actuation(speed)
    coppelia.start_simulation()
    while (t := coppelia.sim.getSimulationTime()) < 3:
        print(f'Simulation time: {t:.3f} [s]')
    coppelia.stop_simulation()
    robot.cleanup()

if __name__ == '__main__':
    main()
