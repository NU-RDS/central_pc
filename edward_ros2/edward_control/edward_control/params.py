import numpy as np

#define physical parameters:
#  L1=0.135
#  L2=0.0527
#  L3=0.321622
#  L4=0.098
L1 = 0.154
L2 = 0.100699
L3 = 0.31274
L4 = 0.112946
L5 = 0.063573
total_length = L1 + L2 + L3 + L4

#  H1=0.032
#  H2=0.007265
#  H3=0.024735

#  W1=0.036329
#  W2=0.0065

H1 = 0.0
H2 = 0.0
H3 = 0.0
W1 = 0.0
W2 = 0.0


m1=2.9665
m2=2.4902
m3=3.3901
m4=0.608
m5=1.5173

#  def find_M(L1,L2,L3,L4,L5):
    #  result=np.array([[1,0,0,-(L1+L2+L3+L4+L5)],
                     #  [0,1,0,-W2],
                     #  [0,0,1,H3],
                     #  [0,0,0,1]])
    #  return result

def find_M(L1,L2,L3,L4,L5):
    result=np.array([[1,0,0,(L1+L2+L3+L4+L5)],
                     [0,1,0,W2],
                     [0,0,1,H3],
                     [0,0,0,1]])
    return result

#  def find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2):
    #  result=np.array([[-1,0,-1,0,-1],
                     #  [0,-1,0,0,0],
                     #  [0,0,0,1,0],
                     #  [0,0,0,-W2,0],
                     #  [0,0,0,L1+L2+L3,H3],
                     #  [0,L1,0,0,W2]])
    #  return result

def find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2):
    result=np.array([[1,0,1,0,1],
                     [0,1,0,0,0],
                     [0,0,0,1,0],
                     [0,0,0,W2,0],
                     [0,0,0,-L1-L2-L3,-H3],
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
#  com_trans = find_com_trans(joint_lst)

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
Blist = find_Blist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2)
M = find_M(L1,L2,L3,L4,L5)
Slist = find_Slist(L1,L2,L3,L4,L5,H1,H2,H3,W1,W2)


