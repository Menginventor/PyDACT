'''
Py DACT Delta Inverse - Forward Kinematics
Based on following assumtion.
1. All diagonal rods length are equal.
2. All prismatic joints (AKA tower) are parallel
'''

import numpy as np
from App.difk_math import *


class dk_param:  # parameter object for DIFK
    def __init__(self,ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len):

        self.ra = ra  # position of tower A or tower X
        self.rb = rb  # position of tower B or tower Y
        self.rc = rc  # position of tower C or tower Z
        self.aa = aa
        self.ab = ab
        self.ac = ac
        self.offsetA = offsetA # carriage position = joint position + offset
        self.offsetB = offsetB
        self.offsetC = offsetC
        self.diag_len = diag_len  #  Diagonal rod length

    def __str__(self):
        reduce_alphaA, reduce_alphaB, reduce_radius = self.reduce()
        param_str = 'ra = '+str(self.ra) + '\r\n' + 'rb = '+str(self.rb) + '\r\n' + 'rc = '+str(self.rc) + '\r\n'  \
                    + 'aa = '+str(np.rad2deg(self.aa)) + ' deg\r\n' \
                    + 'ab = '+str(np.rad2deg(self.ab)) + ' deg\r\n' \
                    + 'ac = '+str(np.rad2deg(self.ac)) + ' deg\r\n' \
                    + 'offsetA = '+str(self.offsetA) + '\r\n'\
                    + 'offsetB = '+str(self.offsetB) + '\r\n'\
                    + 'offsetC = '+str(self.offsetC) + '\r\n' \
                    + 'diag_len = ' + str(self.diag_len) + '\r\n' \
                    + 'reduce_alphaA = ' + str(np.rad2deg(reduce_alphaA)+360) + '\r\n' \
                    + 'reduce_alphaB = ' + str(np.rad2deg(reduce_alphaB)+360) + '\r\n' \
                    + 'reduce_radius = ' + str(reduce_radius) + '\r\n'
        return param_str

    def reduce(self):
        tower_A_pos = np.array([self.ra * np.cos(self.aa), self.ra * np.sin(self.aa)])
        tower_B_pos = np.array([self.rb * np.cos(self.ab), self.rb * np.sin(self.ab)])
        tower_C_pos = np.array([self.rc * np.cos(self.ac), self.rc * np.sin(self.ac)])
        cx,cy,cr = three_points_circle(tower_A_pos, tower_B_pos, tower_C_pos)
        ####
        alpha_A = np.arctan2(tower_A_pos[1] - cy, tower_A_pos[0] - cx)
        alpha_B = np.arctan2(tower_B_pos[1] - cy, tower_B_pos[0] - cx)
        alpha_C = np.arctan2(tower_C_pos[1] - cy, tower_C_pos[0] - cx)
        alpha_offset = np.pi/2 - alpha_C
        reduce_alphaA = alpha_A + alpha_offset
        reduce_alphaB = alpha_B + alpha_offset
        return reduce_alphaA,reduce_alphaB,cr
    @staticmethod
    def set_optimize_var(var):
        ra = var[0]  # position of tower A or tower X
        rb = var[1]  # position of tower B or tower Y
        rc = var[2]  # position of tower C or tower Z
        aa = var[3]
        ab = var[4]
        ac = var[5]
        offsetA = var[6]  # carriage position = joint position + offset
        offsetB = var[7]
        offsetC = var[8]
        diag_len = var[9]  # Diagonal rod length

        return dk_param(ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len)
    @staticmethod
    def array_init(param_arr):
        ra = param_arr[0]  # position of tower A or tower X
        rb = param_arr[1]  # position of tower B or tower Y
        rc = param_arr[2]  # position of tower C or tower Z
        aa = param_arr[3]
        ab = param_arr[4]
        ac = param_arr[5]
        offsetA = param_arr[6]  # carriage position = joint position + offset
        offsetB = param_arr[7]
        offsetC = param_arr[8]
        diag_len = param_arr[9]  # Diagonal rod length
        return dk_param(ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len)

    def get_param_array(self):
        return [self.ra,self.rb,self.rc,self.aa,self.ab,self.ac,self.offsetA,self.offsetB,self.offsetC,self.diag_len]



def DIK (dkp,point):
    # dia_lengrh**2 = r**2 + h**2
    # h**2 = dia_lengrh**2  - r**2
    if point.shape == (3,):
        pass
    elif point.shape[1] == 3 and len(point.shape) == 2:
        return np.array([DIK (dkp,point[i,:]) for i in range(point.shape[0])])
    else:
        raise Exception('DIK point dimension error',point.shape)

    tower_A_pos = np.array([dkp.ra * np.cos(dkp.aa), dkp.ra * np.sin(dkp.aa)])
    tower_B_pos = np.array([dkp.rb * np.cos(dkp.ab), dkp.rb * np.sin(dkp.ab)])
    tower_C_pos = np.array([dkp.rc * np.cos(dkp.ac), dkp.rc * np.sin(dkp.ac)])

    r_A_sq = np.square(tower_A_pos[0] - point[0]) + np.square(tower_A_pos[1] - point[1])
    r_B_sq = np.square(tower_B_pos[0] - point[0]) + np.square(tower_B_pos[1] - point[1])
    r_C_sq = np.square(tower_C_pos[0] - point[0]) + np.square(tower_C_pos[1] - point[1])
    diag_len_sq = np.square(dkp.diag_len)
    # joint position = carriage position + offset
    joint_A = np.sqrt(diag_len_sq - r_A_sq) + point[2] + dkp.offsetA
    joint_B = np.sqrt(diag_len_sq - r_B_sq) + point[2] + dkp.offsetB
    joint_C = np.sqrt(diag_len_sq - r_C_sq) + point[2] + dkp.offsetC

    return np.array([joint_A,joint_B,joint_C])


def DFK(dkp,jointPosition):
    if jointPosition.shape == (3,):
        pass
    elif jointPosition.shape[1] == 3 and  len(jointPosition.shape) == 2:
        return np.array([DFK (dkp,jointPosition[i,:]) for i in range(jointPosition.shape[0])])
    else:
        raise Exception('DFK jointPosition dimension error',jointPosition.shape)

    tower_A_pos = np.array([dkp.ra * np.cos(dkp.aa), dkp.ra * np.sin(dkp.aa)])
    tower_B_pos = np.array([dkp.rb * np.cos(dkp.ab), dkp.rb * np.sin(dkp.ab)])
    tower_C_pos = np.array([dkp.rc * np.cos(dkp.ac), dkp.rc * np.sin(dkp.ac)])
    # carriage position = joint position - offset
    carriagePosA = np.array([tower_A_pos[0],tower_A_pos[1],jointPosition[0] - dkp.offsetA])
    carriagePosB = np.array([tower_B_pos[0],tower_B_pos[1],jointPosition[1] - dkp.offsetB])
    carriagePosC = np.array([tower_C_pos[0],tower_C_pos[1],jointPosition[2] - dkp.offsetC])
    ####

    ####
    carriageFrameHMAT = threePointToHMAT(carriagePosA,carriagePosB,carriagePosC)

    pointA = homoTrans(np.linalg.inv(carriageFrameHMAT),carriagePosA)
    pointB = homoTrans(np.linalg.inv(carriageFrameHMAT), carriagePosB)
    pointC = homoTrans(np.linalg.inv(carriageFrameHMAT), carriagePosC)

    circleX,circleY,circleR = three_points_circle(pointA,pointB,pointC)

    carriageHeight = -np.sqrt(np.square(dkp.diag_len)-np.square(circleR))

    endEffectorPos = homoTrans((carriageFrameHMAT),np.array([circleX,circleY,carriageHeight]))

    return endEffectorPos


def plane_height(plane,point):
    if point.shape == (3,):
        pass
    elif point.shape[1] == 3 and len(point.shape) == 2:
        return np.array([plane_height(plane,point[i,:]) for i in range(point.shape[0])])
    else:
        raise Exception('Point dimension error',point.shape)
    px = point[0]
    py = point[1]
    # from Ax + By + Cz + D = 0
    # then z = -(Ax + By + D)/C
    A = plane[0]
    B = plane[1]
    C = plane[2]
    D = plane[3]
    pz = -(A*px + B*py + D)/C
    return np.array([px,py,pz])


def probing_emulator(point,model_param,actual_param,plane):
    if point.shape == (3,):
        pass
    elif point.shape[1] == 3 and len(point.shape) == 2:
        return np.array([probing_emulator(point[i,:],model_param,actual_param,plane) for i in range(point.shape[0])])
    else:
        raise Exception('Point dimension error',point.shape)

    joint_pos = DIK(model_param,point)
    end_pos = DFK(actual_param,joint_pos)
    end_z_probe = plane_height(plane,end_pos)
    joint_pos_probe = DIK(actual_param,end_z_probe)
    z_probe = DFK(model_param,joint_pos_probe)
    return z_probe

def mesh_plane(x,y,plane):
    X, Y = np.meshgrid(x, y)
    Z = np.zeros(X.shape)
    print('Z shape = ', Z.shape)
    for i in range(x.size):
        for j in range(y.size):
            pnt = np.array([X[i, j], Y[i, j], 0])

            Z[i][j] = plane_height(plane, pnt)[2]
    return X,Y,Z

def dict_to_dk_param(epr_data_dict):
    step_per_mm = epr_data_dict['step_per_mm']
    diag_len = epr_data_dict['diag_length']
    ra = epr_data_dict['horiz_radius'] + epr_data_dict['A_radius']
    rb = epr_data_dict['horiz_radius'] + epr_data_dict['B_radius']
    rc = epr_data_dict['horiz_radius'] + epr_data_dict['C_radius']
    aa = np.deg2rad(epr_data_dict['A_alpha'])
    ab = np.deg2rad(epr_data_dict['B_alpha'])
    ac = np.deg2rad(epr_data_dict['C_alpha'])
    offsetA = epr_data_dict['A_offset']/step_per_mm
    offsetB = epr_data_dict['B_offset']/step_per_mm
    offsetC = epr_data_dict['C_offset']/step_per_mm
    loaded_param = dk_param(ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len)
    return loaded_param