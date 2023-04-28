import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from google.colab import files
import os.path
from scipy import stats
from inverse_dyn import InverseDynamics
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

directory="D:\Profiles\yjc010\Downloads\\"
#this is the directory to store the data
def store_z_tor(z,tor,n):
  z=np.array(z)
  tor=np.array(tor)
  #z and tor should be array of the same length
  data=np.concatenate((np.expand_dims(z, 0), np.expand_dims(tor, 0)), axis=0)
  df = pd.DataFrame(data)
  df.to_csv("z_tor_joint"+str(n)+".csv")
  files.download("z_tor_joint"+str(n)+".csv")
  #df.to_csv(os.path.join(directory,"z_tor_joint"+str(n)+".csv"))
  return df
def get_tor_z(n):
  #function returns z then tor
  data = pd.read_csv(directory+"z_tor_joint"+str(n)+".csv")
  data = data.to_numpy()
  return data[0,1:], data[1,1:]

def z_tor_plot(n):
  #n is the joint number. calling tor_z_plot(1) will show the computed torque vs z plot of joint 1
  z, tor = get_tor_z(n)  
  plt.scatter(z, tor)
  m, b, r, p, std_err = stats.linregress(z, tor)
  plt.plot(z, m * z + b,color="red")
  plt.xlabel('z')
  plt.ylabel('torque')
  plt.title('torque vs z of joint '+str(n)+' slope='+str(m)+' inter='+str(b))
  plt.show()
  return m, b

def z_tor_plot2(z,tor):
  #n is the joint number. calling tor_z_plot(1) will show the computed torque vs z plot of joint 1
  plt.scatter(z, tor)
  m, b, r, p, std_err = stats.linregress(z, tor)
  plt.plot(z, m * z + b,color="red")
  plt.xlabel('z')
  plt.ylabel('torque')
  plt.title('torque vs z'+' slope='+str(m)+' inter='+str(b))
  plt.show()
  return m, b