from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

x1,y1,z1 = [],[],[]
x3,y3,z3 = [],[],[]
path = np.load('.\Backup_variables\path14.npy')
path=path[:-2]
bx = [0,2,3,4]
by = [2,1,3,2]
bz = [-1,-1,-1,-1]
for i in path:
    x1.append(i[0])
    y1.append(i[1])
    z1.append(i[2])

test = [[0,0,0.1],[1,0,0.1],[2,0,0.1],[3,0,0.1],[4,0,0.1],
        [4,1,0.1],[3,1,0.1],[2,1,0.1],[1,1,0.1],[0,1,0.1],
        [0,2,0.1],[1,2,0.1],[2,2,0.1],[3,2,0.1],[4,2,0.1],
        [4,3,0.1],[3,3,0.1],[2,3,0.1],[1,3,0.1],[0,3,0.1],
        [0,4,0.1],[1,4,0.1],[2,4,0.1],[3,4,0.1],[4,4,0.1]]
test = np.asarray(test,dtype=float)
for i in test:
    x3.append(i[0])
    y3.append(i[1])
    z3.append(i[2])
fig = plt.figure(figsize=(5,5))
ax = plt.axes(projection='3d')
ax.set_xlabel(r'$S_X$',labelpad=5)
ax.set_ylabel(r'$S_Y$',labelpad=5)
ax.set_zlabel(r'$S_Z$',labelpad=10)
ax.set_title('Optimum Path vs Naive Path')
ax.xaxis.set_ticks(np.arange(0., 4.1, 1.))
ax.yaxis.set_ticks(np.arange(0.,4.1,1.))
ax.plot3D(x3, y3, z3, 'gray', label = 'Naive Path')
ax.plot3D(x1, y1, z1, 'red', label = 'Optimum Path')
ax.scatter(bx, by, bz, c = 'orange',marker='o', label = 'Orange Balls',s=150)
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles, labels,loc='upper right',bbox_to_anchor=(1.3, 1.1),borderaxespad=0.)



