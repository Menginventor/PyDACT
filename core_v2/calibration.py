from core_v2.pydact_core import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



points = probe_point_config(5,70,12)



model_param = dk_param(64.5,64.5,64.5,np.deg2rad(-150),np.deg2rad(-30),np.deg2rad(90),0,0,0,217)
actual_param = dk_param(60.5,68.5,65.5,np.deg2rad(-152),np.deg2rad(-28),np.deg2rad(90),0,10,-20,217)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax2 = fig.add_subplot(122, projection='3d')



probe_joint_pos = np.load('probe_joint_pos.npy')

probe_point = np.load('probe_point.npy')
probe_z_pos,probe_a_x,probe_a_y = plane_fit(probe_point)
probe_plane_r_mat = np.transpose(np.matmul(rmat_x(probe_a_x),rmat_y(probe_a_y)))
print(probe_point)
est_param,est_plane,sol = param_est_2(model_param,probe_joint_pos)
est_plane_ax = est_plane[0]
est_plane_ay = est_plane[1]
est_plane_r_mat = np.transpose(np.matmul(rmat_x(est_plane_ax),rmat_y(est_plane_ay)))
print('est_param = ')
print(est_param)
#print('actual_param = ')
#print(actual_param)

estPoint = DFK(est_param,probe_joint_pos)

rot_estPoint = np.array([np.matmul(est_plane_r_mat,estPoint[i,:]) for i in range(estPoint.shape[0])])
rot_probePoint = np.array([np.matmul(probe_plane_r_mat,probe_point[i,:])-probe_z_pos for i in range(probe_point.shape[0])])



ax.scatter(rot_probePoint[:,0], rot_probePoint[:,1], rot_probePoint[:,2],color='r')
#ax2.scatter(estPoint[:,0], estPoint[:,1], estPoint[:,2],color='b')
ax.scatter(rot_estPoint[:,0], rot_estPoint[:,1], rot_estPoint[:,2],color='g')
plt.show()
