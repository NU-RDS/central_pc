# -*- coding: utf-8 -*-
"""modIK

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1-6CDlqFkPEiayaDUA1P4F0Vuynn4ePfj
"""

pip install modern_robotics

import modern_robotics as mr
import numpy as np

def modIK(Slist,M, thetalist, dest):
  T = mr.FKinSpace(M, Slist, thetalist)
  v=T[0:3,3]
  return thetalist+np.linalg.pinv(mr.JacobianSpace(Slist, thetalist))@np.concatenate((np.zeros(3),dest[-3:]-v))