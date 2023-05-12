import modern_robotics as Mr
import numpy as np
import matplotlib.pyplot as plt
import mr_helpers as mr


#define physical parameters:
L1=0.135
L2=0.0527
L3=0.321622
L4=0.098
L5=0.063573
H1=0.032
H2=0.007265
H3=0.024735
W1=0.036329
W2=0.0065
m1=2.9665
m2=2.4902
m3=3.3901
m4=0.608
m5=1.5173

def find_M(L1,L2,L3,L4,L5):
    result=np.array([[1,0,0,-(L1+L2+L3+L4+L5)],
                     [0,1,0,-W2],
                     [0,0,1,H3],
                     [0,0,0,1]])
    return result

def find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2):
    result=np.array([[-1,0,-1,0,-1],
                     [0,-1,0,0,0],
                     [0,0,0,1,0],
                     [0,0,0,-W2,0],
                     [0,0,0,L1+L2+L3,H3],
                     [0,L1,0,0,W2]])
    return result

def find_Blist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2):
    result=np.array([[-1,0,-1,0,-1],
                     [0,-1,0,0,0],
                     [0,0,0,1,0],
                     [0,-H3,0,0,0],
                     [-H3,0,-H3,-L4-L5,0],
                     [W2,L2+L3+L4+L5,W2,0,0]])
    return result

def find_com_trans(lst):
    joint1=[[1,0,0,lst[0][0]],
            [0,1,0,lst[0][1]],
            [0,0,1,lst[0][2]],
            [0,0,0,1]]
    joint2=[[0,-1,0,lst[1][0]],
            [1,0,0,lst[1][1]],
            [0,0,1,lst[1][2]],
            [0,0,0,1]]
    joint3=[[0,1,0,lst[2][0]],
            [-1,0,0,lst[2][1]],
            [0,0,1,lst[2][2]],
            [0,0,0,1]]
    joint4=[[0,0,1,lst[3][0]],
            [0,1,0,lst[3][1]],
            [-1,0,0,lst[3][2]],
            [0,0,0,1]]
    joint5=[[0,0,-1,lst[4][0]],
            [0,1,0,lst[4][1]],
            [0,0,1,lst[4][2]],
            [1,0,0,1]]
    return [joint1,joint2,joint3,joint4,joint5]

com_lst=[[-0.106781,0.015551,-0.000092],
           [-0.106425,-0.02849,-0.000163],
           [-0.20732,0.012398,-0.01071],
           [-0.202806,-0.005974,0.019361],
           [-0.118948,0.000157,-0.025392]]
com_trans=find_com_trans(joint_lst))

def find_Glist(m1,m2,m3,m4,m5):
    G1=np.diag([0,0,0,m1,m1,m1])
    G2=np.diag([0,0,0,m2,m2,m2])
    G3=np.diag([0,0,0,m3,m3,m3])
    G4=np.diag([0,0,0,m4,m4,m4])
    G5=np.diag([0,0,0,m5,m5,m5])
    Glist=[G1,G2,G3,G4,G5]
    return Glist

def find_Mlist(com_lst):
    M01=[[1,0,0,lst[0][0]],
         [0,1,0,lst[0][1]],
         [0,0,1,lst[0][2]],
         [0,0,0,1]]
    M12=[[0,-1,0,lst[1][0]],
         [1,0,0,lst[1][1]],
         [0,0,1,lst[1][2]],
         [0,0,0,1]]
    M23=[[0,1,0,lst[2][0]],
         [-1,0,0,lst[2][1]],
         [0,0,1,lst[2][2]],
         [0,0,0,1]]
    M34=[[0,0,1,lst[3][0]],
         [0,1,0,lst[3][1]],
         [-1,0,0,lst[3][2]],
         [0,0,0,1]]
    M45=[[0,0,-1,lst[4][0]],
         [0,1,0,lst[4][1]],
         [0,0,1,lst[4][2]],
         [1,0,0,1]]
    Mlist=[M01,M12,M23,M34,M45]
    return Mlist

#Input calculation for IK:
Blist=find_Blist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2)
M=find_M(L1,L2,L3,L4,L5)
Slist=find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2)

def IKinBody(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err)


def IKinSpace(Slist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the space frame for an open chain robot

    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Slist = np.array([[0, 0,  1,  4, 0,    0],
                          [0, 0,  0,  0, 1,    0],
                          [0, 0, -1, -6, 0, -0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([ 1.57073783,  2.99966384,  3.1415342 ]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Tsb = mr.FKinSpace(M,Slist, thetalist)
    Vs = np.dot(mr.Adjoint(Tsb), \
                mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T))))
    err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
          or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianSpace(Slist, \
                                                          thetalist)), Vs)
        i = i + 1
        Tsb = mr.FKinSpace(M, Slist, thetalist)
        Vs = np.dot(mr.Adjoint(Tsb), \
                    mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T))))
        err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
              or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
    return (thetalist, not err)

#input calculation for ID
Slist=find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2)
Glist=find_Glist(m1,m2,m3,m4,m5)
Mist=find_Mlist(com_lst)
g=np.array([0,0,-9.81])

def InverseDynamics(thetalist, g, Force, Mlist, Glist, Slist):
    """Computes inverse dynamics in the space frame for an open chain robot

    :param thetalist: n-vector of joint variables
        [removed from MR code:]
        :param dthetalist: n-vector of joint rates
        :param ddthetalist: n-vector of joint accelerations
    :param g: Gravity vector g
    :param Ftip: Spatial force applied by the end-effector expressed in frame
                 {n+1}
    :param Mlist: List of link frames {i} relative to {i-1} at the home
                  position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :return: The n-vector of required joint forces/torques
    This function uses forward-backward Newton-Euler iterations to solve the
    equation:
    taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) \
              + g(thetalist) + Jtr(thetalist)Ftip

    Example Input (3 Link Robot):
        thetalist = np.array([0.1, 0.1, 0.1])
        dthetalist = np.array([0.1, 0.2, 0.3])
        ddthetalist = np.array([2, 1.5, 1])
        g = np.array([0, 0, -9.8])
        Ftip = np.array([1, 1, 1, 1, 1, 1])
        M01 = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, 0.089159],
                        [0, 0, 0,        1]])
        M12 = np.array([[ 0, 0, 1,    0.28],
                        [ 0, 1, 0, 0.13585],
                        [-1, 0, 0,       0],
                        [ 0, 0, 0,       1]])
        M23 = np.array([[1, 0, 0,       0],
                        [0, 1, 0, -0.1197],
                        [0, 0, 1,   0.395],
                        [0, 0, 0,       1]])
        M34 = np.array([[1, 0, 0,       0],
                        [0, 1, 0,       0],
                        [0, 0, 1, 0.14225],
                        [0, 0, 0,       1]])
        G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
        G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
        Glist = np.array([G1, G2, G3])
        Mlist = np.array([M01, M12, M23, M34])
        Slist = np.array([[1, 0, 1,      0, 1,     0],
                          [0, 1, 0, -0.089, 0,     0],
                          [0, 1, 0, -0.089, 0, 0.425]]).T
    Output:
        np.array([74.69616155, -33.06766016, -3.23057314])
    """

    '''
    Modified for RDS Robot Arm: ignore inertial parameters; assume the arm is
    quasi-static at all times during its motion; only find expected joint torques due
    to gravity.
    '''
    dthetalist = np.array([0, 0, 0, 0, 0.0])
    ddthetalist = np.array([0, 0, 0, 0, 0.0])

    n = len(thetalist)
    Mi = np.eye(4)
    Ai = np.zeros((6, n))
    AdTi = [[None]] * (n + 1)
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g)]
    AdTi[n] = mr.Adjoint(mr.TransInv(Mlist[n]))
    Fi = np.array(Force).copy()
    taulist = np.zeros(n)

    for i in range(n):
        Mi = np.dot(Mi,Mlist[i])
        Ai[:, i] = np.dot(mr.Adjoint(mr.TransInv(Mi)), np.array(Slist)[:, i])
        AdTi[i] = mr.Adjoint(np.dot(mr.MatrixExp6(mr.VecTose3(Ai[:, i] * \
                                            -thetalist[i])), \
                                 mr.TransInv(Mlist[i])))
        Vi[:, i + 1] = np.dot(AdTi[i], Vi[:,i]) + Ai[:, i] * dthetalist[i]
        Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) \
                       + Ai[:, i] * ddthetalist[i] \
                       + np.dot(mr.ad(Vi[:, i + 1]), Ai[:, i]) * dthetalist[i]
        
    for i in range (n - 1, -1, -1):
        Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) \
             + np.dot(np.array(Glist[i]), Vdi[:, i + 1]) \
             - np.dot(np.array(mr.ad(Vi[:, i + 1])).T, \
                      np.dot(np.array(Glist[i]), Vi[:, i + 1]))
        taulist[i] = np.dot(np.array(Fi).T, Ai[:, i])

    return taulist

    
