from core_v2.pydact_core import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

points = probe_point(5,70,12)

plane = paz_plane(np.array([0,0,0]),np.deg2rad(0.5) ,np.deg2rad(0.5))

model_param = dk_param(64.5,64.5,64.5,np.deg2rad(-150),np.deg2rad(-30),np.deg2rad(90),0,0,0,217)
actual_param = dk_param(65.5,65.5,65.5,np.deg2rad(-150),np.deg2rad(-30),np.deg2rad(90),0,0,0,217)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plane_point = plane_height(plane,points)
probe_point = probing_emulator(points,model_param,actual_param,plane)

plane_fit_res = plane_fit(probe_point)
x_axis = np.linspace(-70, 70, 12)
y_axis = np.linspace(-70, 70, 12)
X, Y = np.meshgrid(x_axis, y_axis)
Z = np.zeros(X.shape)
print('Z shape = ',Z.shape)
for i in range(x_axis.size):
    for j in range(y_axis.size):
        pnt = np.array([X[i,j],Y[i,j],0])

        Z[i][j] = plane_height(plane_fit_res,pnt)[2]

ax.plot_wireframe(X, Y, Z)
print()

ax.scatter(plane_point[:,0], plane_point[:,1], plane_point[:,2],color='b')
ax.scatter(probe_point[:,0], probe_point[:,1], probe_point[:,2],color='r')

plt.show()
