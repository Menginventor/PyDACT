import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
delta = 0.005
x = np.arange(-2.0, 2.0, delta)
y = np.arange(-2.0, 2.0, delta)
X, Y = np.meshgrid(x, y)
Z1 = np.sqrt(X**2 +Y**2)
for i in range(y.size):
    for j in range(x.size):
        if Z1[i,j] >1:
            Z1[i, j] = None
plt.axes().set_aspect('equal')
plt.contourf(x,y,Z1, 20, cmap='jet')
plt.colorbar();
plt.show()