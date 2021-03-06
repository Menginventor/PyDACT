'''
This is main program to read eeprom and probing data and give us the kinematic parameter.

'''
from App.pydact_core import *
from matplotlib import pyplot as plt
import yaml
from mpl_toolkits.mplot3d import Axes3D




epr_dkp = yaml.load(open("epr_dkp.yaml")) #Load parameter
step_per_mm = epr_dkp['step_per_mm']
model_param = dict_to_dk_param(epr_dkp) # store as dk_param object





#
probe_joint_pos = np.load('probe_joint_pos.npy')  # Load probing data

probe_point = np.load('probe_point.npy') # Load joint oisition of probing data

probe_z_pos,probe_a_x,probe_a_y = plane_fit(probe_point)
probe_plane_r_mat = np.transpose(np.matmul(rmat_x(probe_a_x),rmat_y(probe_a_y)))

est_param,est_plane,sol = param_est_2(model_param,probe_joint_pos)  # Calculate parameter (from pydact core)
est_plane_ax = est_plane[0]
est_plane_ay = est_plane[1]
est_plane_r_mat = np.transpose(np.matmul(rmat_x(est_plane_ax),rmat_y(est_plane_ay)))

print('model_param = ')
print(model_param)
print('est_param = ')
print(est_param)
print('plane',np.rad2deg(est_plane_ax),np.rad2deg(est_plane_ax))
# carriage position = joint position + offset
#norm tower offset
min_offset = min([est_param.offsetA,est_param.offsetB,est_param.offsetC])
norm_offsetA = est_param.offsetA-min_offset
norm_offsetB = est_param.offsetB-min_offset
norm_offsetC = est_param.offsetC-min_offset
####
norm_offsetA_step = norm_offsetA * step_per_mm
norm_offsetB_step = norm_offsetB * step_per_mm
norm_offsetC_step = norm_offsetC * step_per_mm
# Use this for tower offset
print('norm_offsetA_step',norm_offsetA_step)
print('norm_offsetB_step',norm_offsetB_step)
print('norm_offsetC_step',norm_offsetC_step)
#
print(norm_offsetA,norm_offsetB,norm_offsetC)
######## Just for plot
print('model_cost',param_cost(model_param,probe_joint_pos))
print('est_cost',param_cost(est_param,probe_joint_pos))
estPoint = DFK(est_param,probe_joint_pos)
dfkPoint = DFK(model_param,probe_joint_pos)
rot_estPoint = np.array([np.matmul(est_plane_r_mat,estPoint[i,:]) for i in range(estPoint.shape[0])])
rot_probePoint = np.array([np.matmul(probe_plane_r_mat,probe_point[i,:])-probe_z_pos for i in range(probe_point.shape[0])])

####Plotting####
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(probe_point[:,0], probe_point[:,1], probe_point[:,2],color='r')
#ax2.scatter(estPoint[:,0], estPoint[:,1], estPoint[:,2],color='b')
ax.scatter(rot_estPoint[:,0], rot_estPoint[:,1], rot_estPoint[:,2],color='g')
plt.show()
