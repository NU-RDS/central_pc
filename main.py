
from inverse_dyn import InverseDynamics
from bot_parameters import *
import numpy as np

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