import math
import numpy as np
from scipy.spatial.distance import squareform, pdist
from numpy.linalg import norm

width, height = 640, 480

class Boids:

    def __init__(self, N):
        # initialize the boid simulation
        self.pos = height * np.random.rand(2 * N).reshape(N, 2)
        angles = 2 * math.pi * np.random.rand(N)
        self.vel = np.array(list(zip(np.sin(angles), np.cos(angles))))
        self.N = N
        self.velFix = 2.0
        self.minDist = 15
        self.maxDist = 55
        #weights
        self.al = 1.0
        self.se = 1.0
        self.co = 1.0
        self.maxRuleVel = 0.5
        self.maxVel = 2.0
        self.noise = 0.2
        self.aveVel = 0
        self.aveDist = 0
    def move(self):
        # update the simulation by one time step
        self.distMatrix = squareform(pdist(self.pos))
        self.aveDist = self.distMatrix.sum() / (self.N) ** 2
        # apply rules
        self.vel += self.applyRules()
        self.fix(self.vel, self.velFix)
        # 加上噪声
        angles = 2 * math.pi * np.random.rand(self.N)
        self.vel += np.array(list(zip(np.sin(angles), np.cos(angles)))) * self.noise * self.velFix
        # 控制速度一定
        self.fix(self.vel, self.velFix)
        self.aveVel = norm(self.vel.sum(axis=0) / self.N)
        self.pos += self.vel
        # 周期边界条件
        self.applyBC()
    def limitVec(self, vec, maxVal):
        mag = norm(vec)

        if mag > maxVal:
            vec[0], vec[1] = vec[0] * maxVal / mag, vec[1] * maxVal / mag
    def limit(self, X, maxVal):
        # limit the magnitide of 2D vectors in array X to maxVal
        for vec in X:
            self.limitVec(vec, maxVal)
    def fixVec(self, vec, velFix):
        # limit the magnitide of the 2D vector
        mag = norm(vec)
        vec[0], vec[1] = vec[0] * velFix / mag, vec[1] * velFix / mag
    def fix(self, X, velFix):
        # limit the magnitide of 2D vectors in array X to maxVal
        for vec in X:
            self.fixVec(vec, velFix)

#边界条件
    def applyBC(self):
        # apply boundary conditions
        deltaR = -0.
        for coord in self.pos:
            if coord[0] > width + deltaR:
                coord[0] = -deltaR

            if coord[0] < -deltaR:
                coord[0] = width + deltaR

            if coord[1] > height + deltaR:
                coord[1] = -deltaR

            if coord[1] < -deltaR:
                coord[1] = height + deltaR
#boids运动的三个条件：
    #Separation: boids move away from other boids that are too close
    #Alignment: boids attempt to match the velocities of their neighbors
    #Cohesion: boids move toward the center of mass of their neighbors
    def applyRules(self):
        # rule 1:separation
        D = self.distMatrix < self.minDist
        # 求解质心位置，并且产生远离质心的速度
        vel = self.pos * D.sum(axis=1).reshape(self.N, 1) - D.dot(self.pos)
        vel = vel * self.se
        self.limit(vel, self.maxRuleVel)
        D = self.distMatrix < self.maxDist

        # rule 2:alignment
        # 求范围内平均速度
        vel2 = D.dot(self.vel)
        self.limit(vel2, self.maxRuleVel)
        vel += vel2 * self.al

        # rule 3: cohesion
        # 朝着质心移动
        vel3 = D.dot(self.pos) - self.pos  # 质心-自己位置
        self.limit(vel3, self.maxRuleVel)
        vel += vel3 * self.co
        return vel