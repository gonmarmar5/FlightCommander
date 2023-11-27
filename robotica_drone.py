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

    num_sonar = 16
    sonar_max = 1.0

    def __init__(self, sim, robot_id):
        self.sim = sim
        print('*** getting handles', robot_id)

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

        self.d = sim.getObject(f'/{robot_id}/base')

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

            if self.simulateParticles:
                self.particleObjects[i] = self.sim.addParticleObject(
                    self.ttype, self.particleSize, self.particleDensity, [2, 1, 0.2, 3, 0.4],
                    self.particleLifeTime, self.maxParticleCount, [0.3, 0.7, 1]
                )

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

        if self.fakeShadow:
            self.shadowCont = self.sim.addDrawingObject(
                self.sim.drawing_discpts + self.sim.drawing_cyclic +
                self.sim.drawing_25percenttransparency + self.sim.drawing_50percenttransparency +
                self.sim.drawing_itemsizes, 0.2, 0, -1, 1
            )

    def cleanup(self):
        self.sim.removeDrawingObject(self.shadowCont)
        for i in range(len(self.particleObjects)):
            self.sim.removeParticleObject(self.particleObjects[i])


    def actuation(self, speed):
        pos = self.sim.getObjectPosition(self.d, self.sim.handle_world)
        if self.fakeShadow:
            itemData = [pos[0], pos[1], 0.002, 0, 0, 0, 1, 0.2]
            self.sim.addDrawingObjectItem(self.shadowCont, itemData)

        # Vertical control:
        targetPos = self.sim.getObjectPosition(self.d, self.sim.handle_world)
        targetPos = np.array(targetPos) + np.array(speed)
        pos = self.sim.getObjectPosition(self.d, self.sim.handle_world)
        l = self.sim.getVelocity(self.heli)
        e = (targetPos[2] - pos[2])

        self.cumul = self.cumul + e
        pv = self.pParam * e
        thrust = 5.45 + pv + self.iParam * self.cumul + self.dParam * (e - self.lastE) + l[0][0] * self.vParam
        self.lastE = e

        # Horizontal control:
        sp = speed
        m = self.sim.getObjectMatrix(self.d, self.sim.handle_world)
        vx = [1, 0, 0]
        vx = self.sim.multiplyVector(m, vx)
        vy = [0, 1, 0]
        vy = self.sim.multiplyVector(m, vy)
        alphaE = (vy[2] - m[11])
        alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - self.pAlphaE)
        betaE = (vx[2] - m[11])
        betaCorr = -0.25 * betaE - 2.1 * (betaE - self.pBetaE)
        self.pAlphaE = alphaE
        self.pBetaE = betaE
        alphaCorr = alphaCorr + sp[2] * 0.005 + 1 * (sp[2] - self.psp2)
        betaCorr = betaCorr - sp[0] * 0.005 - 1 * (sp[0] - self.psp1)
        self.psp2 = sp[2]
        self.psp1 = sp[0]

        # Rotational control:
        euler = np.zeros(3) - np.array(speed)
        rotCorr = euler[2] * 0.1 + 2 * (euler[2] - self.prevEuler)
        self.prevEuler = euler[2]

        print(thrust)
        print(alphaCorr)
        print(betaCorr)
        print(rotCorr)

        # Decide of the motor velocities:
        self.handle_propeller(1, thrust * (1 - alphaCorr + betaCorr + rotCorr))
        self.handle_propeller(2, thrust * (1 - alphaCorr - betaCorr - rotCorr))
        self.handle_propeller(3, thrust * (1 + alphaCorr - betaCorr + rotCorr))
        self.handle_propeller(4, thrust * (1 + alphaCorr + betaCorr - rotCorr))


    def handle_propeller(self, index, particle_velocity):
        propellerRespondable = self.propellerHandles[index - 1]
        propellerJoint = self.jointHandles[index - 1]
        propeller = self.sim.getObjectParent(propellerRespondable)
        particleObject = self.particleObjects[index - 1]
        maxParticleDeviation = math.tan(
            self.particleScatteringAngle * 0.5 * math.pi / 180) * particle_velocity
        notFullParticles = 0

        t = self.sim.getSimulationTime()
        self.sim.setJointPosition(propellerJoint, t * 10)
        ts = self.sim.getSimulationTimeStep()

        m = self.sim.getObjectMatrix(propeller, self.sim.handle_world)
        particleCnt = 0
        pos = [0, 0, 0]
        dir = [0, 0, 1]

        requiredParticleCnt = self.particleCountPerSecond * ts + notFullParticles
        notFullParticles = requiredParticleCnt % 1
        requiredParticleCnt = math.floor(requiredParticleCnt)

        #while particleCnt < requiredParticleCnt:
        #    x = (random.random() - 0.5) * 2
        #    y = (random.random() - 0.5) * 2
        #    if x * x + y * y <= 1:
        #        if self.simulateParticles:
        #            pos[0] = x * 0.08
        #            pos[1] = y * 0.08
        #            pos[2] = -self.particleSize * 0.6
        #            dir[0] = pos[0] + \
        #                (random.random() - 0.5) * maxParticleDeviation * 2
        #            dir[1] = pos[1] + \
        #                (random.random() - 0.5) * maxParticleDeviation * 2
        #            dir[2] = pos[2] - particle_velocity * \
        #                (1 + 0.2 * (random.random() - 0.5))
        #            pos = self.sim.multiplyVector(m, pos)
        #            dir = self.sim.multiplyVector(m, dir)
        #            itemData = [pos[0], pos[1], pos[2], dir[0], dir[1], dir[2]]
        #            self.sim.addParticleObjectItem(particleObject, itemData)
        #        particleCnt += 1

        # Apply a reactive force onto the body:
        totalExertedForce = (particleCnt * self.particleDensity * particle_velocity *
                            math.pi * self.particleSize * self.particleSize *
                            self.particleSize) / (6 * ts)
        force = [0, 0, totalExertedForce]
        m[4] = 0
        m[8] = 0
        m[11] = 0
        force = self.sim.multiplyVector(m, force)
        rotDir = 1 - np.mod(index, 2) * 2
        torque = [0, 0, rotDir * 0.002 * particle_velocity]
        torque = self.sim.multiplyVector(m, torque)
        self.sim.addForceAndTorque(propellerRespondable, force, torque)


def main(args=None):
    coppelia = Coppelia()
    robot = QuadCopter(coppelia.sim, 'Quadcopter')
    robot.actuation()
    coppelia.start_simulation()
    while (t := coppelia.sim.getSimulationTime()) < 3:
        print(f'Simulation time: {t:.3f} [s]')
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()
