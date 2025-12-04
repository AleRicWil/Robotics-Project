"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022

v2.0 updates:
- Added optional 'gripper' flag to SerialArm init for arms where the last joint is a gripper (handled separately in hardware).
- Added comments for integration with real robot control (e.g., convert to degrees).
- In fk: Comment noting gripper handling.
- In ik_position: Optional 'return_deg' param to return q in degrees (using transforms.rad_to_deg); note for gripper.
"""
from transforms_v2_0 import *
import numpy as np

eye = np.eye(4)
pi = np.pi


class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts one link of dh parameters and returns a function "f" that will generate a
    homogeneous transform "A" given "q" as an input. A represents the transform from 
    link i to link i+1

    Parameters:
    dh - 1 x 4 list or iterable of floats, dh parameter table for one transform from link i to link i+1,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the transform from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            def A(q):
                theta = dh[0] + q
                d = dh[1]
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                cth = np.cos(theta)
                sth = np.sin(theta)
                cal = np.cos(alpha)
                sal = np.sin(alpha)

                return np.array(
                    [[cth, -sth * cal, sth *sal, a * cth],
                     [sth, cth * cal, -cth * sal, a * sth],
                     [0, sal, cal, d],
                     [0, 0, 0, 1]])


        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                theta = dh[0]
                d = dh[1] + q
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                cth = np.cos(theta)
                sth = np.sin(theta)
                cal = np.cos(alpha)
                sal = np.sin(alpha)

                return np.array(
                    [[cth, -sth * cal, sth * sal, a * cth],
                     [sth, cth * cal, -cth * sal, a * sth],
                     [0, sal, cal, d],
                     [0, 0, 0, 1]])


        self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """


    def __init__(self, dh, jt=None, base=eye, tip=eye, joint_limits=None, gripper=False):
        """
        arm = SerialArm(dh, joint_type, base=I, tip=I, radians=True, joint_limits=None, gripper=False)
        :param dh: n length list or iterable of length 4 list or iterables representing dh parameters, [theta d a alpha]
        :param jt: n length list or iterable of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy or sympy array representing SE3 transform from world frame to frame 0
        :param tip: 4x4 numpy or sympy array representing SE3 transform from frame n to tool frame
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        :param gripper: If True, indicates the last joint is a gripper (handled separately in hardware; optional).
        """
        self.dh = dh
        self.n = len(dh)
        self.gripper = gripper  # Flag for gripper; if True, last joint may be excluded from some computations for hardware integration.

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # generating the function A(q) for each set of DH parameters
        for i in range(self.n):
            # TODO use the class definition above (dh2AFunc), and the dh parameters and joint type to
            # make a function and then append that function to the "transforms" list. 
            f = dh2AFunc(dh[i], self.jt[i])
            self.transforms.append(f.A)

        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits

        # calculating rough numbers to understand the workspace for drawing the robot
        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][1]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += np.linalg.norm(np.array([dh[1], dh[2]]))



    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string


    def fk(self, q, index=None, base=False, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the homogeneous transform from the specified frames.
                If index is a single integer "i", returns the transform from frame 0 to frame i.
                If index is a list of two integers [i,j], returns the transform from frame i to frame j.
                Defaults to the full transform from frame 0 to frame n.
                If base is True, includes the base transform pre-multiplied to the result.
                If tip is True, includes the tip transform post-multiplied to the result.

            Parameters:
            q - n length numpy array or list of joint coordinates
            index - None, int, or length 2 list of ints, specifies the frames for the output transform
            base - bool, whether to include the base transform or not
            tip - bool, whether to include the tip transform or not

            Returns:
            T - 4x4 numpy array, homogeneous transform
        """

        q = list(q)

        if index is None:
            index = self.n
        if type(index) == int:
            i = 0
            j = index
        else:
            i = index[0]
            j = index[1]

        T = np.eye(4)
        for k in range(i, j):
            T = T @ self.transforms[k](q[k])

        if base:
            T = self.base @ T

        if tip:
            T = T @ self.tip

        # Note: If self.gripper, the last joint (q[-1]) may be handled separately in hardware (e.g., not part of pose FK).

        return T


    def fk_all(self, q, base=False, tip=False):
        """
        T = arm.fk_all(q, base=False, tip=False)
        Description:
        Returns all of the frame transforms from frame 0 to frame i for i = 1 to n

        Parameters:
        q - n length numpy array or list of joint coordinates
        base - bool, whether to include the base transform or not
        tip - bool, whether to include the tip transform or not

        Returns:
        T - n length list of 4x4 numpy arrays
        """

        q = list(q)

        T = []
        T_cur = np.eye(4)

        for k in range(0, self.n):
            T_cur = T_cur @ self.transforms[k](q[k])
            T.append(T_cur)

        if base:
            for i in range(self.n):
                T[i] = self.base @ T[i]

        if tip:
            T[-1] = T[-1] @ self.tip

        return T


    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q, index=None, base=False, tip=False)
        Description:
        Returns the geometric jacobian representing the differential kinematics
        relating the joint velocities to the cartesian velocity of the point
        defined by the transform returned from fk(q, index, base, tip)

        Parameters:
        q - n length numpy array or list of joint coordinates
        index - None, int, or length 2 list of ints, specifies the frames for the output transform
        base - bool, whether to include the base transform or not
        tip - bool, whether to include the tip transform or not

        Returns:
        J - 6 x n numpy array, geometric jacobian
        """

        q = list(q)
        J = np.zeros((6, self.n))

        if index is None:
            index = self.n
        if type(index) == int:
            i = 0
            j = index
        else:
            i = index[0]
            j = index[1]

        T = []
        T_cur = np.eye(4)

        for k in range(0, j):
            T_cur = T_cur @ self.transforms[k](q[k])
            T.append(T_cur)

        if base:
            for k in range(0, j):
                T[k] = self.base @ T[k]

        p = T[-1][0:3, 3]

        for k in range(i, j):
            if self.jt[k] == 'r':
                z_axis = T[k-1][0:3, 2]
                p_cur = T[k-1][0:3, 3]
                J[0:3, k] = np.cross(z_axis, p - p_cur)
                J[3:6, k] = z_axis
            else:
                J[0:3, k] = T[k-1][0:3, 2]
                J[3:6, k] = np.zeros(3)

        if tip:
            J[0:3, :] = self.tip[0:3, 0:3] @ J[0:3, :]
            J[3:6, :] = self.tip[0:3, 0:3] @ J[3:6, :]

        return J


    def ik_position(self, target, q0=None, method='pinv', tol=1e-6, max_iter=1000, K=None, kd=0.01, 
                    force=False, debug=False, debug_step=False, return_deg=False):
        """
        q, error, count, success, message = arm.ik(target, q0=None, method='pinv', tol=1e-6, max_iter=1000, K=None, kd=0.01, force=False, debug=False, debug_step=False, return_deg=False)
        Description:
        Attempts to find a joint configuration "q" that results in a desired end-effector position given by "target"
        The method can be either 'pinv' for damped pseudo-inverse, or 'J_T' for Jacobian Transpose

        Parameters:
        target - 3x1 numpy array, the desired end-effector position in frame 0
        q0 - n length numpy array, the seed joint configuration, defaults to zeros if none given
        method - string, either 'pinv' or 'J_T'
        tol - float, the position error tolerance for convergence
        max_iter - int, the maximum number of iterations before returning a failure
        K - 3x3 numpy array, the positive definite gain matrix, defaults to eye(3)
        kd - float, the positive damping factor for the damped pseudo-inverse method
        force - bool, whether to force a solution even if target is out of workspace
        debug - bool, whether to display a visualization of the process
        debug_step - bool, whether to wait for user input between iterations in debug mode
        return_deg - bool, if True, returns q in degrees (for direct hardware integration)

        Returns:
        q - n length numpy array, the joint configuration that achieves the desired position
        error - 3x1 numpy array, the position error at the end of the iteration
        count - int, the number of iterations taken to converge or fail
        success - bool, whether the solution converged to the desired tolerance or not
        message - string, the exit message of the function, either converged or max iterations reached
        """

        if K is None:
            K = np.eye(3)

        # set default to zeros if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        # Try basic check for if the target is in the workspace.
        # Maximum length of the arm is sum(sqrt(d_i^2 + a_i^2)), distance to target is norm(A_t)
        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)

        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"
        else:
            if target_distance > maximum_reach:
                print("Target out of workspace, but finding closest solution anyway")
            else:
                print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
                    float(target_distance), float(maximum_reach)))

        if not isinstance(K, np.ndarray):
            return q, error, count, False,  "No gain matrix 'K' provided"

        count = 0

        def get_error(q):
            cur_position = self.fk(q)
            e = target - cur_position[0:3, 3]
            return e

        def get_jacobian(q):
            J = self.jacob(q)
            return J[0:3, :]

        def get_jdag(J):
            Jdag = J.T @ np.linalg.inv(J @ J.T + np.eye(3) * kd**2)
            return Jdag

        e = get_error(q)

        if debug == True: 
            from visualization_v2_0 import VizScene
            import time
            arm = SerialArm(self.dh, self.jt, self.base, self.tip)
            viz = VizScene()
            viz.add_arm(arm)
            
            # this arm with joints that are almost pink is for the intermediate solutions
            viz.add_arm(arm, joint_colors=[np.array([1.0, 51.0/255.0, 1.0, 1])]*arm.n)


        while np.linalg.norm(e) > tol and count < max_iter:
            count = count + 1
            J = get_jacobian(q) 

            if method == 'J_T':
                qdelta = J.T @ K @ e 
            elif method == 'pinv':
                Jdag = get_jdag(J)
                qdelta = Jdag @ K @ e
            else:
                return q, False, "that method is not implemented"
            
            # here we assume that delta_t has been included in the gain matrix K. 
            q = q + qdelta

            if debug==True: 
                viz.update(qs=[q0, q])
                if debug_step == True:
                    input('press Enter to see next iteration')
                else: 
                    time.sleep(1.0/2.0)

            e = get_error(q)
            print("error is: ", np.linalg.norm(e), "\t count is: ", count)

        if debug==True: 
            viz.close_viz()

        if self.gripper:
            print("Note: Gripper angle not included in IK; handle separately in hardware scripts.")

        if return_deg:
            q = np.degrees(q)  # Convert to degrees for hardware integration

        return (q, e, count, count < max_iter, 'No errors noted, all clear')


    def Z_shift(self, R=np.eye(3), p=np.zeros(3,), p_frame='i'):

        """
        Z = Z_shift(R, p, p_frame_order)
        Description: 
            Generates a shifting operator (rotates and translates) to move twists and Jacobians 
            from one point to a new point defined by the relative transform R and the translation p. 

        Parameters:
            R - 3x3 numpy array, expresses frame "i" in frame "j" (e.g. R^j_i)
            p - 3x1 numpy array length 3 iterable, the translation from the initial Jacobian point to the final point, expressed in the frame as described by the next variable.
            p_frame - is either 'i', or 'j'. Allows us to define if "p" is expressed in frame "i" or "j", and where the skew symmetrics matrix should show up. 

        Returns:
            Z - 6x6 numpy array, can be used to shift a Jacobian, or a twist
        """
        from scipy.linalg import block_diag

        def skew(p):
            return np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
        
        # generate our skew matrix
        S = skew(p)
        buf = np.eye(6)
        buf[0:3,3:] = -S

        if p_frame == 'i':
            Z = block_diag(R, R) @ buf
        elif p_frame == 'j':
            Z = buf @ block_diag(R, R)
        else:
            Z = None

        return Z



if __name__ == "__main__":
    from visualization_v2_0 import VizScene
    import time

    # Defining a table of DH parameters where each row corresponds to another joint.
    # The order of the DH parameters is [theta, d, a, alpha] - which is the order of operations. 
    # The symbolic joint variables "q" do not have to be explicitly defined here. 
    # This is a two link, planar robot arm with two revolute joints. 
    dh = [[0, 0, 0.3, 0],
          [0, 0, 0.3, 0]]

    # make robot arm (assuming all joints are revolute)
    arm = SerialArm(dh)

    # defining joint configuration
    q = [pi/4.0, pi/4.0]  # 45 degrees and 45 degrees

    # show an example of calculating the entire forward kinematics
    Tn_in_0 = arm.fk(q)
    print("Tn_in_0:\n", Tn_in_0, "\n")
    #show_frame('0', '2', Tn_to_0) # this will only work if all of values are numeric

    # show an example of calculating the kinematics between frames 0 and 1
    T1_in_0 = arm.fk(q, index=[0,1])
    print("T1_in 0:\n", T1_in_0, "\n")
    #show_frame('0', '1', T1_to_0)

    print(arm)

    viz = VizScene()

    viz.add_frame(arm.base, label='base')
    viz.add_frame(Tn_in_0, label="Tn_in_0")
    viz.add_frame(T1_in_0, label="T1_in_0")

    time_to_run = 30
    refresh_rate = 60

    for i in range(refresh_rate * time_to_run):
        viz.update()
        time.sleep(1.0/refresh_rate)
    
    viz.close_viz()