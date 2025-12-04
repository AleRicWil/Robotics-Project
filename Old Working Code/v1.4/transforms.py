"""
Transforms Module - Contains code to describe SO(3) and SE(3) 

Key for HW 04
"""

import numpy as np
from numpy import sin, cos, sqrt
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """
    R = np.array([[cos(th), -sin(th)],
                  [sin(th), cos(th)]])
    return clean_rotation_matrix(R)

## 3D Rotations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    R = np.array([[1, 0, 0],
                  [0, cos(th), -sin(th)],
                  [0, sin(th), cos(th)]])
    return clean_rotation_matrix(R)


def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    R = np.array([[cos(th), 0, sin(th)],
                  [0, 1, 0],
                  [-sin(th), 0, cos(th)]])
    return clean_rotation_matrix(R)


def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """
    R = np.array([[cos(th), -sin(th), 0],
                  [sin(th), cos(th), 0],
                  [0, 0, 1]])
    return clean_rotation_matrix(R)


# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''

    return R.T


def clean_rotation_matrix(R, eps=1e-12):
    '''
    This function is not required, but helps to make sure that all
    matrices we return are proper rotation matrices
    '''

    for i in range(R.shape[0]):
        for j in range(R.shape[1]):
            if np.abs(R[i, j]) < eps:
                R[i, j] = 0.
            elif np.abs(R[i, j] - 1) < eps:
                R[i, j] = 1.
    return R
 
def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """

    T = np.eye(4)
    T[0:3,0:3] = R
    T[0:3, 3] = p

    return T

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    T_inv = np.eye(4)

    R = T[0:3,0:3]
    p = T[0:3, 3]
    R_inv = R.T
    p_inv = -R.T @ p
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = p_inv

    return T_inv

def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """

    roll = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = np.arctan2(R[2, 1], R[2, 2])

    return np.array([roll, pitch, yaw])


def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.acos," "np.sin," etc. 

    ang = np.arccos(0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1))
    axis_angle = np.array([ang,
                          (R[2, 1] - R[1, 2]) / (2 * sin(ang)),
                          (R[0, 2] - R[2, 0]) / (2 * sin(ang)),
                          (R[1, 0] - R[0, 1]) / (2 * sin(ang))])
    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """

    # follow formula in book, see equation (2.25) on pg. 54
    rx = v[0]
    ry = v[1]
    rz = v[2]
    c = np.cos(ang)
    s = np.sin(ang)
    R = np.array([[rx**2 * (1-c) + c, rx*ry*(1-c)-rz*s, rx*rz*(1-c)+ry*s],
                  [rx*ry*(1-c)+rz*s, ry**2 * (1-c) + c, ry*rz*(1-c)-rx*s],
                  [rx*rz*(1-c)-ry*s, ry*rz*(1-c)+rx*s, rz**2 * (1-c) + c]])

    return clean_rotation_matrix(R)

def R2quat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R - 3x3 numpy array

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """

    return np.array([0.5*sqrt(np.abs(R[0, 0] + R[1, 1] + R[2, 2] + 1)),
                     0.5*np.sign(R[2, 1] - R[1, 2]) * sqrt(np.abs(R[0, 0] - R[1, 1] - R[2, 2] + 1)),
                     0.5*np.sign(R[0, 2] - R[2, 0]) * sqrt(np.abs(R[1, 1] - R[2, 2] - R[0, 0] + 1)),
                     0.5*np.sign(R[1, 0] - R[0, 1]) * sqrt(np.abs(R[2, 2] - R[0, 0] - R[1, 1] + 1))])
                    
def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """

    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]

    R =  np.array([[2*(nu**2+ex**2)-1, 2*(ex*ey-nu*ez), 2*(ex*ez+nu*ey)],
                    [2*(ex*ey+nu*ez), 2*(nu**2+ey**2)-1, 2*(ey*ez-nu*ex)],
                    [2*(ex*ez-nu*ey), 2*(ey*ez+nu*ex), 2*(nu**2+ez**2)-1]])
    
    return clean_rotation_matrix(R)


def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """
 
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return clean_rotation_matrix(R)


def R2euler(R, order='xyz'):

    D = dict(x=(rotx, 0), y=(roty, 1), z=(rotz, 2))

    rotA, axis1 = D[order[0]]
    rotB, axis2 = D[order[1]]
    rotC, axis3 = D[order[2]]

    if axis1 >= axis3:
        s = -1
    else:
        s = 1

    Ri = np.eye(3)
    Rf = R

    v = np.cross(Rf[:, axis3], (s * Ri[:, axis1]))
    if np.isclose(norm(v), 0):  # This indicates a rotation about the A axis ONLY.
        th1 = np.arccos(Ri[:, axis2] @ (Rf[:, axis2]))
        th2 = 0
        th3 = 0
        Ri = Ri @ rotA(th1)
    else:
        v = v / norm(v)
        th1 = np.arccos(min(max(Ri[:, axis2] @ v, 1), 0))
        Ri = Ri @ rotA(th1)

        th2 = np.arccos(min(max(Ri[:, axis3] @ Rf[:, axis3], 1), 0))
        Ri = Ri @ rotB(th2)

        th3 = np.arccos(min(max(Ri[:, axis2] @ Rf[:, axis2], 1), 0))
        Ri = Ri @ rotC(th3)

    return np.array([th1, th2, th3])
