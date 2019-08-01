from core_v2.core_dfik import *
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

def param_est_cost(var,*argv):
    init_param = argv[0]
    joints_pos = argv[1]
    ra = var[0]
    rb = var[1]
    rc = var[2]
    aa = var[3]
    ab = var[4]
    ac = var[5]
    offsetA = var[6]
    offsetB = var[7]
    offsetC = var[8]
    a_x = var[9]
    a_y = var[10]

    diag_len = init_param.diag_len
    param = dk_param(ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len)
    end_pos = DFK(param, joints_pos)
    plane = paz_plane(np.array([0, 0, 0]), a_x, a_y)
    dist_ssq = 0
    plane_vec = plane[0:3]
    d = plane[3]
    for i in range(end_pos.shape[0]):
        dist_ssq += np.square(np.dot(plane_vec[:], end_pos[i, :]) + d)
    print(dist_ssq)
    return dist_ssq

def param_est(init_param,joint_pos):
    x0 = np.array([init_param.ra,init_param.rb,init_param.rc,init_param.aa,
          init_param.ab,init_param.ac,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    sol = optimize.minimize(param_est_cost,x0=x0,args=(init_param,joint_pos),method='trust-constr')
    ###
    var = sol.x
    ra = var[0]
    rb = var[1]
    rc = var[2]
    aa = var[3]
    ab = var[4]
    ac = var[5]
    offsetA = var[6]
    offsetB = var[7]
    offsetC = var[8]
    a_x = var[9]
    a_y = var[10]

    diag_len = init_param.diag_len
    param = dk_param(ra,rb,rc,aa,ab,ac,offsetA,offsetB,offsetC,diag_len)
    plane = paz_plane(np.array([0, 0, 0]), a_x, a_y)
    return param,plane,sol
####
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
    for i in range(end_pos.shape[0]):
        dist_ssq += np.square(np.dot(plane_vec[:], end_pos[i, :]) + d)
    print(dist_ssq)
    return dist_ssq

def param_est_2(init_param,joint_pos):
    x0 = np.array([init_param.ra,init_param.aa,
          init_param.ab,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    sol = optimize.minimize(param_est_cost_2,x0=x0,args=(init_param,joint_pos),method='trust-constr')
    ###
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
    plane = paz_plane(np.array([0, 0, 0]), a_x, a_y)
    return param,(a_x, a_y),sol

if __name__ == '__main__':
    pass