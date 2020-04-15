#!/usr/bin/env python3
import matplotlib
import matplotlib.pyplot as plt
from math import atan2

"""
You're free to use the code as you wish, I'm writing this at
like 4 am for some unknown reason, but it's fun! Still some weird
behavior I want to investigate and things to add, but this is a
little demo of how adaptive proportional navigation works
"""


def vecScalarMult(vec, scalar):
    scalarVec = [scalar for _ in range(len(vec))]
    for i in range(len(vec)):
        scalarVec[i] = vec[i] * scalarVec[i]
    return scalarVec


def vecAdd(vec1, vec2):
    vec3 = []
    for x, y in zip(vec1, vec2):
        vec3.append(x + y)
    return vec3


def vecSub(vec1, vec2):
    vec3 = []
    for x, y in zip(vec1, vec2):
        vec3.append(x - y)
    return vec3


def vecMag(vec):
    mag = 0
    for x in vec:
        mag += x**2
    return mag**0.5


def vecNormalize(vec):
    sumElements = 0
    for x in vec:
        sumElements += x
    if sumElements != 0:
        for i in range(len(vec)):
            vec[i] /= sumElements
    return vec


def vecDot(vec1, vec2):
    dot = 0
    for x, y in zip(vec1, vec2):
        dot += x*y
    return dot


class DynamicObject:
    def __init__(self, pos, vel, dt):
        self.prevPos = pos
        self.pos = pos
        self.vel = vel
        self.dt = dt

    def updatePos(self, accel=(0,0,0)):
        halfDtSqr = 0.5*(self.dt**2)
        self.prevPos = self.pos
        self.pos = vecAdd(vecScalarMult(self.vel, self.dt), self.pos)
        self.pos = vecAdd(vecScalarMult(accel, halfDtSqr), self.pos)
        self.vel = vecAdd(vecScalarMult(accel, self.dt), self.vel)

    def isCollidedWith(self, otherObj, threshold=1):
        return vecMag(vecSub(self.pos, otherObj.pos)) <= threshold


class Missile(DynamicObject):
    def __init__(self, pos, vel, dt, maxAccel):
        super().__init__(pos, vel, dt)
        self.maxAccel = maxAccel

    def calcAugmentedPN(self, tgtObj, N=None, Nt=None):
        # Based off https://www.moddb.com/members/blahdy/blogs/gamedev-introduction-to-proportional-navigation-part-i
        # and http://trajectorysolution.com/HomingGuidance.html
        if N is None:
            N = 3.0
        if Nt is None:
            #Nt = 9.8 * self.dt # TODO add more physics stuff
            Nt = [0,0,0]

        xOld, yOld, zOld = vecSub(tgtObj.prevPos, self.prevPos)
        xNew, yNew, zNew = vecSub(tgtObj.pos, self.pos)

        # Compute rate of change of line of sight angle
        dXAngle = (atan2(zNew, xNew) - atan2(zOld, xOld)) / self.dt
        dYAngle = (atan2(xNew, yNew) - atan2(xOld, yOld)) / self.dt
        dZAngle = (atan2(xNew, zNew) - atan2(xOld, zOld)) / self.dt
        LOS_delta = [-dXAngle, -dYAngle, -dZAngle]

        # Compute closing velocity
        Vc = vecSub(vecSub(tgtObj.pos, self.pos),
                    vecSub(tgtObj.prevPos, self.prevPos))
        Vc = vecMag(vecScalarMult(Vc, 1 / self.dt))

        # Calculate acceleration along LOS delta (not technically normal-acceleration,
        # should see about that)
        accel = vecScalarMult(LOS_delta, N * Vc)
        accel = vecAdd(accel, vecScalarMult(Nt, N * 0.5))

        if vecMag(accel) > self.maxAccel:
            accel = self.constrainAccel(accel)

        return accel

    def constrainAccel(self, accel):
        ratio = self.maxAccel / vecMag(accel)
        for i in range(len(accel)):
            accel[i] *= ratio
        return accel

    def missed(self, tgtObj):
        prevDist = vecMag(vecSub(self.prevPos, tgtObj.prevPos))
        dist = vecMag(vecSub(self.pos, tgtObj.pos))
        # TODO make this better
        return dist > prevDist


def getMissileAndTgt(dt):
    maxAccel = 30*9.8
    pos = [float(x) for x in input('Enter missile initial pos:').split(',')]
    vel = [float(x) for x in input('Enter missile initial vel:').split(',')]
    checkInputParams(pos, vel)
    missile = Missile(pos, vel, dt, maxAccel)

    pos = [float(x) for x in input('Enter target initial pos:').split(',')]
    vel = [float(x) for x in input('Enter target initial vel:').split(',')]
    checkInputParams(pos, vel)
    target = DynamicObject(pos, vel, dt)
    
    return missile, target


def checkInputParams(pos, vel):
    if len(pos) != 3 or len(vel) != 3:
        raise Exception


def addToLog(logData, missile, target):
    misX, misY, misZ = missile.pos
    tarX, tarY, tarZ = target.pos
    logData.append((i, misX, misY, misZ, tarX, tarY, tarZ))


def getMinAndMax(missile, target, currentMin, currentMax):
    for x in missile.pos:
        if x > currentMax:
            currentMax = x
        elif x < currentMin:
            currentMin = x

    for x in target.pos:
        if x > currentMax:
            currentMax = x
        elif x < currentMin:
            currentMin = x

    return currentMin, currentMax


def plotData(missileLst, targetLst, minLim, maxLim):
    missileX, missileY, missleZ = missileLst
    targetX, targetY, missleZ = targetLst

    ax = plt.axes(projection='3d')
    ax.set_xlim3d(minLim, maxLim)
    ax.set_ylim3d(minLim, maxLim)
    ax.set_zlim3d(minLim, maxLim)
    ax.plot3D(missileX, missileY, missileZ, 'gray')
    ax.plot3D(missileX[-1:], missileY[-1:], missileZ[-1:], 'r*')
    ax.plot3D(targetX, targetY, targetZ, 'blue')
    ax.plot3D(targetX[-1:], targetY[-1:], targetZ[-1:], 'r*')
    plt.show()


def saveData(logData, filename):
    with open(filename, 'w') as f:
        for entry in logData:
            f.write('{}\n'.format(','.join((str(x) for x in entry))))


if __name__ == '__main__':
    try:
        i = 0
        dt = 0.01
        accel = (0,0,0)
        thresh = 10
        minVal, maxVal = float('inf'), float('-inf')
        missile, target = getMissileAndTgt(dt)
        missileX, missileY, missileZ = [], [], []
        targetX, targetY, targetZ = [], [], []
        logData = []

        while not missile.isCollidedWith(target, threshold=thresh) and not missile.missed(target):
            misX, misY, misZ = missile.pos
            missileX.append(misX)
            missileY.append(misY)
            missileZ.append(misZ)

            tarX, tarY, tarZ = target.pos
            targetX.append(tarX)
            targetY.append(tarY)
            targetZ.append(tarZ)

            minVal, maxVal = getMinAndMax(missile, target, minVal, maxVal)

            addToLog(logData, missile, target)

            target.updatePos()
            missile.updatePos(accel)
            accel = missile.calcAugmentedPN(target)

            if i % 100 == 0:
                print('Time {}, missile at {}, target at {}'.format(i*dt, missile.pos, target.pos))
            i += 1

        addToLog(logData, missile, target)

        print('\n' + '-'*80 + '\n')
        if missile.isCollidedWith(target, threshold=thresh):
            print('Missile collided with target at location {}!'.format(target.pos))
            print('Missile location: {}'.format(missile.pos))
        else:
            print('Missed... target at {}, missile at {}'.format(target.pos, missile.pos))

        plotData((missileX, missileY, missileZ),
                 (targetX, targetY, targetZ),
                 minVal, maxVal)

        saveData(logData, 'log.txt')
    except Exception as e:
        import pdb; pdb.set_trace()
