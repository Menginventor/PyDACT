import numpy as np
from core.pydact_math import *
class delta_kinematic_param:
    def __init__(self, delta_radius, alpha_a, alpha_b,offset_a,offset_b,diag_length):
        self.diag_length = diag_length
        self.delta_radius = delta_radius
        self.alpha_a = alpha_a
        self.alpha_b = alpha_b
        self.offset_a = offset_a
        self.offset_b = offset_b
    def __str__(self):
        return 'diag_length = '+str(self.diag_length)+', delta_radius = '+str(self.delta_radius)\
               +', alpha_a = '+str(np.rad2deg(self.alpha_a))+', alpha_b = '+str(np.rad2deg(self.alpha_b))
    def argv(self):
        return [self.delta_radius, self.alpha_a, self.alpha_b,self.offset_a,self.offset_b]
    @staticmethod
    def init_form_printer(diag_length,radius_a, radius_b, radius_c, alpha_a, alpha_b, alpha_c,offset_a,offset_b,offset_c):
        towerA_pos = np.array([radius_a * np.cos(np.deg2rad(alpha_a)), radius_a * np.sin(np.deg2rad(alpha_a))])
        towerB_pos = np.array([radius_b * np.cos(np.deg2rad(alpha_b)), radius_b * np.sin(np.deg2rad(alpha_b))])
        towerC_pos = np.array([radius_c * np.cos(np.deg2rad(alpha_c)), radius_c * np.sin(np.deg2rad(alpha_c))])
        c_x,c_y,c_r = three_points_circle(towerA_pos, towerB_pos, towerC_pos)
        new_alpha_a = np.arctan2(towerA_pos[1] - c_y, towerA_pos[0] - c_x)
        new_alpha_b = np.arctan2(towerB_pos[1] - c_y, towerB_pos[0] - c_x)
        new_alpha_c = np.arctan2(towerC_pos[1] - c_y, towerC_pos[0] - c_x)
        ####
        print('new_alpha_c =',new_alpha_c)
        alpha_offset = np.pi/2.0 - new_alpha_c
        norm_alpha_a = new_alpha_a + alpha_offset
        norm_alpha_b = new_alpha_b + alpha_offset
        ####
        return delta_kinematic_param(c_r, norm_alpha_a, norm_alpha_b,offset_a-offset_c,offset_b-offset_c,diag_length)

    @staticmethod
    def init_argv(argv,diag_length):
        delta_radius = argv[0]
        alpha_a = argv[1]
        alpha_b = argv[2]
        offset_a = argv[3]
        offset_b = argv[4]

        return delta_kinematic_param(delta_radius, alpha_a, alpha_b,offset_a,offset_b,diag_length)


def DIK (dkp,point):
    # dia_lengrh**2 = r**2 + h**2
    # h**2 = dia_lengrh**2  - r**2

    if point.shape == (3,):
        pass
    elif point.shape[1] == 3 and len(point.shape) == 2:
        return np.array([DIK (dkp,point[i,:]) for i in range(point.shape[0])])
    else:
        print('DIK point dimension error',point.shape)
    tower_A_pos = np.array([dkp.delta_radius*np.cos(dkp.alpha_a),dkp.delta_radius*np.sin(dkp.alpha_a)])
    tower_B_pos = np.array([dkp.delta_radius * np.cos(dkp.alpha_b), dkp.delta_radius * np.sin(dkp.alpha_b)])
    tower_C_pos = np.array([0, dkp.delta_radius])
    r_A_sq = np.square(tower_A_pos[0] - point[0]) + np.square(tower_A_pos[1] - point[1])
    r_B_sq = np.square(tower_B_pos[0] - point[0]) + np.square(tower_B_pos[1] - point[1])
    r_C_sq = np.square(tower_C_pos[0] - point[0]) + np.square(tower_C_pos[1] - point[1])
    diag_len_sq = np.square(dkp.diag_length)
    #
    joint_A = np.sqrt(diag_len_sq - r_A_sq) + point[2] + dkp.offset_a
    joint_B = np.sqrt(diag_len_sq - r_B_sq) + point[2] + dkp.offset_b
    joint_C = np.sqrt(diag_len_sq - r_C_sq) + point[2]
    return np.array([joint_A,joint_B,joint_C])

def DFK(dkp,jointPosition):
    if jointPosition.shape == (3,):
        pass
    elif jointPosition.shape[1] == 3 and  len(jointPosition.shape) == 2:
        return np.array([DFK (dkp,jointPosition[i,:]) for i in range(jointPosition.shape[0])])
    else:
        print('DFK jointPosition dimension error',jointPosition.shape)

    tower_A_pos = np.array([dkp.delta_radius * np.cos(dkp.alpha_a), dkp.delta_radius * np.sin(dkp.alpha_a)])
    tower_B_pos = np.array([dkp.delta_radius * np.cos(dkp.alpha_b), dkp.delta_radius * np.sin(dkp.alpha_b)])
    tower_C_pos = np.array([0, dkp.delta_radius])

    carriagePosA = np.array([tower_A_pos[0],tower_A_pos[1],jointPosition[0] - dkp.offset_a])
    carriagePosB = np.array([tower_B_pos[0],tower_B_pos[1],jointPosition[1] - dkp.offset_b])
    carriagePosC = np.array([tower_C_pos[0],tower_C_pos[1],jointPosition[2] ])
    ####

    ####
    carriageFrameHMAT = threePointToHMAT(carriagePosA,carriagePosB,carriagePosC)

    pointA = homoTrans(LA.inv(carriageFrameHMAT),carriagePosA)
    pointB = homoTrans(LA.inv(carriageFrameHMAT), carriagePosB)
    pointC = homoTrans(LA.inv(carriageFrameHMAT), carriagePosC)


    circleX,circleY,circleR = three_points_circle(pointA,pointB,pointC)

    carriageHeight = -np.sqrt(np.square(dkp.diag_length)-np.square(circleR))

    endEffectorPos = homoTrans((carriageFrameHMAT),np.array([circleX,circleY,carriageHeight]))

    return endEffectorPos