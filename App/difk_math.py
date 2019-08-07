'''
Mathematic function for Delta Inverse-Forward Kinematic (DIFK)
'''
import numpy as np
'''
Transformation
'''
def three_points_circle(p1,p2,p3):
    # derrive from (x-x_i)**2 + (y-y_i)**2 = r**2
    # x**2 -2*x*x_i + x_i**2 + y**2 -2*y*y_i + y_i**2 - r**2 = 0
    # -2*x_i*x -2*y_i*y + (x**2 + y**2 - r**2) = -x_i**2 -y_i**2
    # Y = A*X, X = A^(-1)*Y
    points = np.array([p1,p2,p3])
    coff_mat = np.array([[np.multiply(-2.00, pt[0]), np.multiply(-2.00, pt[1]), -1] for pt in points])
    coff_mat_inv = np.linalg.inv(coff_mat)
    Y_mat = np.array([-np.square(pt[0]) - np.square(pt[1]) for pt in points])
    X_mat = np.matmul(coff_mat_inv, Y_mat)
    circleX = X_mat[0]
    circleY = X_mat[1]
    circleR = np.sqrt(X_mat[2] + np.square(circleX) + np.square(circleY))
    return circleX,circleY,circleR

def threePointToPlane(p1,p2,p3):
    ABC = np.cross(p2 - p1, p3 - p1)

    ABC = ABC/np.linalg.norm(ABC)
    D = -np.dot(ABC, p1)
    plane = [ABC[0],ABC[1],ABC[2],D]
    return plane

def threePointToHMAT(p1,p2,p3):
    plane = threePointToPlane(p1,p2,p3)

    plane_vec = np.array([plane[0],plane[1],plane[2]])

    plane_const = -plane[3]

    plane_center = np.array([0, 0, (plane_const / plane_vec[2])])

    plane_z_axis = plane_vec / np.linalg.norm(plane_vec)
    plane_y_axis = (p3 - plane_center) / np.linalg.norm((p3 - plane_center))
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

'''
Rotation Matrix
'''
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

'''
Plane
    Plane is represent by 4 parameters A,B,C,D where Ax + By + Cz + D = 0
'''
def pn_plane(point,normal_vec):  # contruct plane from point and normal vector
    # <A,B,C> = Normal vector
    A = normal_vec[0]
    B = normal_vec[1]
    C = normal_vec[2]
    # Ax + By + Cz = -D
    x = point[0]
    y = point[1]
    z = point[2]
    D = -(A*x + B*y + C*z)
    return np.array([A,B,C,D])

def paz_plane(point,a_x,a_y):  # contruct plane from point and angle of z-axis
    z_axis = np.array([0, 0, 1])
    r_mat = np.matmul(rmat_x(a_x),rmat_y(a_y))
    normal_vec = np.matmul(r_mat,z_axis)
    return pn_plane(point,normal_vec)
