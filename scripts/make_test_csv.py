#!/usr/bin/env python3

'''
Generates a CSV file of joint trajectories, useful for testing
'''
import numpy as np

N = 300

# moves joints 1,3,4 from 0 to pi/3 radians in N steps,
# joints 0 and 5 stay at 0
traj = np.array([np.linspace(0,np.pi/3,N) for _ in range(5)]).T
traj[:,0] = np.zeros(N)
traj[:,2] = np.zeros(N)

np.savetxt("test_traj.csv",traj,delimiter=",", fmt="%.4f")
