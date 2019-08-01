import numpy as np
from scipy import  optimize
from numpy import linalg as LA

def rmat_x (theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rmat_y (theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rmat_z (theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
def plane_fit_cost(plane,*argv):
    # plane in format [a,b,c,d] where ax+by+cz+d = 0 and <a,b,c> is unit vactor
    # points in format [[x,y,z],...]
    # plane-point distance = |ax+by+cz+d|
    alpha = plane[0]
    beta = plane[1]
    d = plane[2]
    dist_ssq = 0 # sum of square distance
    points = np.array(argv)
    if len(points.shape)>2:
        points = points[0,:,:]
    z_vec = np.array([[0],[0],[1]])
    r_mat = np.matmul(rmat_x(alpha),rmat_y(beta))
    plane_vec = np.matmul(r_mat,z_vec)
    for i in range(points.shape[0]):
        dist_ssq += np.square(np.dot(plane_vec[:,0],points[i,:]) + d)
    return dist_ssq
def plane_fit_error(plane,points):

    dist = []  # array of distance

    if len(points.shape) > 2:
        points = points[0, :, :]


    for i in range(points.shape[0]):
        dist.append(np.dot(plane[0:3], points[i, :]) + plane[3])
    return max(dist),min(dist)

def plane_fit(points): #  points in format [[x,y,z],...]
    bounds = [(-np.pi,np.pi),(-np.pi,np.pi),(None,None)]
    # plane format in [alpha,beta,d]
    res = optimize.shgo(plane_fit_cost,bounds,points)
    if res.success:
        alpha = res.x[0]
        beta = res.x[1]
        d = res.x[2]
        z_vec = np.array([[0], [0], [1]])
        r_mat = np.matmul(rmat_x(alpha), rmat_y(beta))
        plane_vec = np.matmul(r_mat, z_vec)

        return np.array([plane_vec[0,0],plane_vec[1,0],plane_vec[2,0],d]),res.fun
    else:
        return None,None

####
def three_points_circle(p1,p2,p3):
    # derrive from (x-x_i)**2 + (y-y_i)**2 = r**2
    # x**2 -2*x*x_i + x_i**2 + y**2 -2*y*y_i + y_i**2 - r**2 = 0
    # -2*x_i*x -2*y_i*y + (x**2 + y**2 - r**2) = -x_i**2 -y_i**2
    # Y = A*X, X = A^(-1)*Y
    points = np.array([p1,p2,p3])
    coff_mat = np.array([[np.multiply(-2.00, pt[0]), np.multiply(-2.00, pt[1]), -1] for pt in points])
    coff_mat_inv = LA.inv(coff_mat)
    Y_mat = np.array([-np.square(pt[0]) - np.square(pt[1]) for pt in points])
    X_mat = np.matmul(coff_mat_inv, Y_mat)
    circleX = X_mat[0]
    circleY = X_mat[1]
    circleR = np.sqrt(X_mat[2] + np.square(circleX) + np.square(circleY))
    return circleX,circleY,circleR

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