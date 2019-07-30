'''
Delta Forward-Inverse Kinematics
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np
from numpy import linalg as LA

def carriagePos(alpha,radius,zPos):
    carriagePosX = np.multiply(np.cos(alpha),radius)
    carriagePosY = np.multiply(np.sin(alpha), radius)
    carriagePosZ = zPos
    return np.array([carriagePosX, carriagePosY, carriagePosZ])

def threePointToPlane(p1,p2,p3):
    ABC = np.cross(p2 - p1, p3 - p1)

    ABC = ABC/LA.norm(ABC)
    D = -np.dot(ABC, p1)
    plane = [ABC[0],ABC[1],ABC[2],D]
    return plane

def threePointToHMAT(p1,p2,p3):
    plane = threePointToPlane(p1,p2,p3)

    plane_vec = np.array([plane[0],plane[1],plane[2]])

    plane_const = -plane[3]

    plane_center = np.array([0, 0, (plane_const / plane_vec[2])])

    plane_z_axis = plane_vec / LA.norm(plane_vec)
    plane_y_axis = (p3 - plane_center) / LA.norm((p3 - plane_center))
    plane_x_axis = np.cross(plane_y_axis, plane_z_axis)
    plane_rmat = np.transpose(np.array([plane_x_axis, plane_y_axis, plane_z_axis]))
    plane_hmat = np.zeros((4, 4))

    for row in range(4):
        for col in range(4):
            if row < 3 and col < 3:
                plane_hmat[row][col] = plane_rmat[row][col]
            elif col == 3:
                if row == 3:
                    plane_hmat[row][col] = 1
                else:
                    plane_hmat[row][col] = plane_center[row]
            else:
                plane_hmat[row][col] = 0
    return plane_hmat

def homoTrans(HMAT,vec):
    return np.matmul(HMAT, np.array([vec[0],vec[1],vec[2],1]))[0:3]

def DeltaForwardKinematics(jointPosition,configDict,plotter = None):
    #jointPosition = np.array([configDict['zPosA'],configDict['zPosB'],configDict['zPosC']])
    alpha = np.deg2rad(np.array([configDict['alphaA'], configDict['alphaB'], configDict['alphaC']]))

    deltaRadius = np.array([configDict['deltaRadiusA'], configDict['deltaRadiusB'], configDict['deltaRadiusC']])
    diagonalRodLength = configDict['diagonalRodLength']
    print('jointPosition',jointPosition)
    carriagePosA = carriagePos(alpha[0],deltaRadius[0], jointPosition[0])
    carriagePosB = carriagePos(alpha[1], deltaRadius[1], jointPosition[1])
    carriagePosC = carriagePos(alpha[2], deltaRadius[2], jointPosition[2])
    ####
    #print('carriagePos',carriagePosA, carriagePosB, carriagePosC)
    ####
    carriageFrameHMAT = threePointToHMAT(carriagePosA,carriagePosB,carriagePosC)
    plotFrameHMAT(plotter,carriageFrameHMAT)
    pointA = homoTrans(LA.inv(carriageFrameHMAT),carriagePosA)
    pointB = homoTrans(LA.inv(carriageFrameHMAT), carriagePosB)
    pointC = homoTrans(LA.inv(carriageFrameHMAT), carriagePosC)
    points = [pointA,pointB,pointC]
    #print('point = ',points)
    mat1 = np.array([[np.multiply(-2.00,pt[0]),np.multiply(-2.00,pt[1]),-1] for pt in points ])
    mat1_inv = LA.inv(mat1) #Y = AX , X = A^-1 Y
    matY = np.array([-np.square(pt[0])-np.square(pt[1]) for pt in points])
    matX = np.matmul(mat1_inv, matY)
    circleX = matX[0]
    circleY = matX[1]
    circleR = np.sqrt(matX[2]+np.square(circleX)+np.square(circleY))

    carriageHeight = -np.sqrt(np.square(diagonalRodLength)-np.square(circleR))
    #print(carriageHeight)
    endEffectorPos = homoTrans((carriageFrameHMAT),np.array([circleX,circleY,carriageHeight]))


    plotLine(plotter,homoTrans((carriageFrameHMAT),np.array([circleX,circleY,0])),endEffectorPos,color='r')
    return endEffectorPos

def DeltaInverseKinematics(endeffectorPosition,configDict,plotter = None):
    #jointPosition = np.array([configDict['zPosA'],configDict['zPosB'],configDict['zPosC']])
    alpha = np.deg2rad(np.array([configDict['alphaA'], configDict['alphaB'], configDict['alphaC']]))

    deltaRadius = np.array([configDict['deltaRadiusA'], configDict['deltaRadiusB'], configDict['deltaRadiusC']])
    diagonalRodLength = configDict['diagonalRodLength']

    towerPosA = carriagePos(alpha[0],deltaRadius[0],endeffectorPosition[2])
    towerPosB = carriagePos(alpha[1], deltaRadius[1], endeffectorPosition[2])
    towerPosC = carriagePos(alpha[2], deltaRadius[2], endeffectorPosition[2])
    jointPosition = np.array([
        [np.sqrt(np.square(diagonalRodLength) - np.square(LA.norm(endeffectorPosition- towerPosA))) + endeffectorPosition[2]],
        [np.sqrt(np.square(diagonalRodLength) - np.square(LA.norm(endeffectorPosition - towerPosB))) + endeffectorPosition[2]],
        [np.sqrt(np.square(diagonalRodLength) - np.square(LA.norm(endeffectorPosition - towerPosC))) + endeffectorPosition[2]],
    ])
    ####
    #print('carriagePos',carriagePosA, carriagePosB, carriagePosC)
    ####
    return jointPosition

def kinematicRecheck(jointPosition,configDict,plotter = None):
    #jointPosition = np.array([configDict['zPosA'], configDict['zPosB'], configDict['zPosC']])
    alpha = np.deg2rad(np.array([configDict['alphaA'], configDict['alphaB'], configDict['alphaC']]))

    deltaRadius = np.array([configDict['deltaRadiusA'], configDict['deltaRadiusB'], configDict['deltaRadiusC']])
    diagonalRodLength = configDict['diagonalRodLength']
    carriagePosA = carriagePos(alpha[0], deltaRadius[0], jointPosition[0])
    carriagePosB = carriagePos(alpha[1], deltaRadius[1], jointPosition[1])
    carriagePosC = carriagePos(alpha[2], deltaRadius[2], jointPosition[2])
    endPos = DeltaForwardKinematics(jointPosition,configDict,plotter)
    ####
    plotLine(plotter,endPos,carriagePosA)
    plotLine(plotter, endPos, carriagePosB)
    plotLine(plotter, endPos, carriagePosC)
    ####
    #print(endPos)
    distA = np.linalg.norm(endPos - carriagePosA)
    distB = np.linalg.norm(endPos - carriagePosB)
    distC = np.linalg.norm(endPos - carriagePosC)
    print(distA,distB,distC)

def plotLine(ploter,l1,l2,color='b'):
    ploter.plot([ l1[0],l2[0]],[l1[1],l2[1]],[l1[2],l2[2]],color)

def plotFrameHMAT(ploter,HMAT,length = 25):
    #print(HMAT)
    xAxis = np.array([length, 0, 0])
    yAxis = np.array([0, length, 0])
    zAxis = np.array([0, 0, length])
    p0 = [x[3] for x in HMAT[0:3]]

    x1 =  homoTrans(HMAT,xAxis)
    y1 =  homoTrans(HMAT,yAxis)
    z1 =  homoTrans(HMAT,zAxis)

    plotLine(ploter,p0,x1,color='r')
    plotLine(ploter, p0, y1, color='g')
    plotLine(ploter, p0, z1, color='b')

