from core_v2.pydact_core import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



points = probe_point_config(5,70,12)

ref_plane = paz_plane(np.array([0,0,0]),np.deg2rad(0.5) ,np.deg2rad(0.5))

model_param = dk_param(64.5,64.5,64.5,np.deg2rad(-150),np.deg2rad(-30),np.deg2rad(90),0,0,0,217)
actual_param = dk_param(60.5,68.5,65.5,np.deg2rad(-152),np.deg2rad(-28),np.deg2rad(90),0,10,-20,217)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plane_point = plane_height(ref_plane,points)
probe_point = probing_emulator(points,model_param,actual_param,ref_plane)


x_axis = np.linspace(-70, 70, 12)
y_axis = np.linspace(-70, 70, 12)
ref_planeX,ref_planeY,ref_planeZ = mesh_plane(x_axis,y_axis,ref_plane)

joint_pos = DIK(model_param,probe_point)

est_param,est_plane,sol = param_est_2(model_param,joint_pos)
print('est_param = ')
print(est_param)
print('actual_param = ')
print(actual_param)
estX,estY,estZ = mesh_plane(x_axis,y_axis,est_plane)
estPoint = DFK(est_param,joint_pos)

#ax.plot_wireframe(ref_planeX, ref_planeY, ref_planeZ,color='r')
#ax.plot_wireframe(estX, estY, estZ, color='b')

ax.scatter(probe_point[:,0], probe_point[:,1], probe_point[:,2],color='r')
ax.scatter(estPoint[:,0], estPoint[:,1], estPoint[:,2],color='b')

plt.show()
