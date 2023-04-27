
from inverse_dyn import InverseDynamics
from inverse_kin import IKinSpace

from bot_parameters import *
import numpy as np

def iktest():

    Slist_ik = np.array([[0, 0,  1,  4, 0,    0],
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

    #test of IKInSpace()
    out = IKinSpace(Slist_ik, M, T, thetalist0, eomg, ev)
    print(out)
    #should be (np.array([ 1.57073783,  2.99966384,  3.1415342 ]), True)

def idtest():
    pos = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
    Force = np.array([1, 1, 1, 1, 1, 1]) #wrench applied by the n+1 link to the outside world in its own body frame

#array of joint torques produced by ID
tor = InverseDynamics(pos, gravity, Force, Mlist, Glist, Slist)


def torTobias(tor):
  #TODO: fill in what this function represents
  pass
  return 0

def callpos():
  #TODO: fill in what this function represents
  pass
  return 0

if __name__ == '__main__':
