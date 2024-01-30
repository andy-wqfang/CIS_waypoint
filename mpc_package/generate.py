import csv
import numpy as np 

# Four main reference points

ptA = np.array([-187.1, 212.2])
ptB = np.array([187.2, 212.2])
ptC = np.array([-187.1, 212.2])
ptD = np.array([-187.1, -212.2])

# Important parameters

r1 = 33.3 # center to road side
r2 = 43.9 # center to inner lane's center
width_block = 177.6
height_block = 194.2
width_lane = 21.15

# initialize the nodes
nodes = np.zeros((100, 2))

nodes[1] = ptA + np.array([0, r2])
print(nodes[1])

nodes[2] = ptA + np.array([width_block - 2*r1, r2])
print(nodes[2])

nodes[5] = ptA + np.array([-r2, 0])
nodes[6] = nodes[2] + np.array([r2, -r2])

