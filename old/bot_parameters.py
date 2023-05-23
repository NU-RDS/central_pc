import numpy as np
#following are the fixed parameters of the robot arm

#M's are the transformation matrix between each link at its home configuration, (i to i-1 frame)
gravity = np.array([0, 0, -9.8])
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
M45 = np.array([[1, 0, 0,        0],
                [0, 1, 0,        0],
                [0, 0, 1,        0],
                [0, 0, 0,        1]])
M56 = np.array([[1, 0, 0,        0],
                [0, 1, 0,        0],
                [0, 0, 1,        0],
                [0, 0, 0,        1]])

G1 = np.diag([0, 0, 0, 3.7, 3.7, 3.7])
G2 = np.diag([0, 0, 0, 8.393, 8.393, 8.393])
G3 = np.diag([0, 0, 0, 2.275, 2.275, 2.275])
G4 = np.diag([0, 0, 0, 2.275, 2.275, 2.275])
G5 = np.diag([0, 0, 0, 2.275, 2.275, 2.275])

Glist = np.array([G1, G2, G3, G4, G5])
Mlist = np.array([M01, M12, M23, M34, M45, M56])
Slist = np.array([[1, 0, 0,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0,     0]]).T
#screw axis in space frame

#cutoffs for linear and rotational error for Numerical Inverse Kinematics 
eomg = 0.01
ev = 0.001
