'''
This is core program to finding kinematic parameter.

'''
from App.core_dfik import *
from scipy import  optimize
import numpy as np

def probe_point_config(probe_height,probe_radius,probe_num):
    x_axis = np.linspace(-probe_radius,probe_radius,probe_num)
    y_axis = np.linspace(-probe_radius, probe_radius, probe_num)
    points = np.array( [[x,y,probe_height] for x in x_axis for y in y_axis if x**2 + y**2 <= probe_radius**2] )
    return points

def plane_fit_cost(opt_var, points):
    a_x = opt_var[0]
    a_y = opt_var[1]
    z_pos = opt_var[2]
    plane = paz_plane(np.array([0,0,z_pos]),a_x,a_y)
    dist_ssq = 0
    plane_vec = plane[0:3]
    d = plane[3]
    for i in range(points.shape[0]):
        dist_ssq += np.square(np.dot(plane_vec[:],points[i,:]) + d)
    return dist_ssq


def plane_fit(points):
    sol = optimize.minimize(plane_fit_cost, x0=np.array([0,0,0]),args=points)
    print(sol)
    a_x = sol.x[0]
    a_y = sol.x[1]
    z_pos = sol.x[2]
    return z_pos,a_x,a_y


def param_est_cost_2(var,*argv):
    init_param = argv[0]
    joints_pos = argv[1]
    radius = var[0]

    alpha_a = var[1]
    alpha_b = var[2]

    offsetA = var[3]
    offsetB = var[4]
    offsetC = var[5]
    a_x = var[6]
    a_y = var[7]

    diag_len = init_param.diag_len
    param = dk_param(radius,radius,radius,alpha_a,alpha_b,np.pi/2,offsetA,offsetB,offsetC,diag_len)
    end_pos = DFK(param, joints_pos)
    plane = paz_plane(np.array([0, 0, 0]), a_x, a_y)
    dist_ssq = 0
    plane_vec = plane[0:3]
    d = plane[3]
    plane_r_mat = np.transpose(np.matmul(rmat_x(a_x), rmat_y(a_y)))
    rot_end_pos = np.array([np.matmul(plane_r_mat, end_pos[i, :]) for i in range(end_pos.shape[0])])
    for i in range(end_pos.shape[0]):
        dist_ssq += np.square(rot_end_pos[i,2])
    print(dist_ssq)
    return dist_ssq
def param_cost(init_param,joint_pos):
    x0 = np.array([init_param.ra,init_param.aa,
          init_param.ab,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    return param_est_cost_2(x0,init_param,joint_pos)
def param_est_2(init_param,joint_pos):
    reduce_alphaA, reduce_alphaB, reduce_radius = init_param.reduce()
    x0 = np.array([reduce_radius,reduce_alphaA,
                   reduce_alphaB,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    bounds = [(None,None) for i in range(8)]

    bounds[0] = (60,150)
    bounds[1] = (np.deg2rad(210-10), np.deg2rad(210+10))
    bounds[2] = (np.deg2rad(330 - 10), np.deg2rad(330 + 10))
    bounds[3] = (-1000, 1000)
    bounds[4] = (-1000, 1000)
    bounds[5] = (-1000, 1000)
    bounds[6] = (np.deg2rad(-2),np.deg2rad(2))
    bounds[7] = (np.deg2rad(-2), np.deg2rad(2))
    print('bounds',bounds)
    method = 'Nelder-Mead'
    method = 'trust-constr'
    method = 'SLSQP'
    sol = optimize.minimize(param_est_cost_2,bounds=bounds,x0=x0,args=(init_param,joint_pos),method=method)

    print(sol)
    var = sol.x
    radius = var[0]

    alpha_a = var[1]
    alpha_b = var[2]

    offsetA = var[3]
    offsetB = var[4]
    offsetC = var[5]
    a_x = var[6]
    a_y = var[7]

    diag_len = init_param.diag_len
    param = dk_param(radius,radius,radius,alpha_a,alpha_b,np.pi/2,offsetA,offsetB,offsetC,diag_len)

    return param,(a_x, a_y),sol

if __name__ == '__main__':
    pass