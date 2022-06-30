# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 09:32:38 2022

@author: EliasC
"""

import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import MaxNLocator
import matplotlib.tri as mtri

from scipy.spatial import Delaunay
from RobotDelta import RobotDelta

import math
DEG_TO_RAD = math.pi/180

Robot = RobotDelta()

limart_inf=1
limart_sup=90
a=np.arange(-30*DEG_TO_RAD,90*DEG_TO_RAD,10*DEG_TO_RAD)


# a=-30:10:90;
index=1
index1=1
N=len(a)**3
vol=np.zeros((3,N))
vol1=np.zeros((3,N))

for i in range(limart_inf,len(a),1):
    for j in range(limart_inf,len(a),1):
        for k in range(limart_inf,len(a),1):
            
                vol1[:,index]= Robot.forward_Kinematics(a[i],a[j],a[k])
                
                if vol1[2,index]<= -0.4:
                     vol[:,index1]=vol1[:,index]
                     index1=index1+1
            
            
                index=index+1
                
DT = Delaunay(vol.T)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_trisurf(vol[0,:], vol[1,:], vol[2,:], triangles=DT.simplices, 
                cmap=plt.cm.jet, color='blue', linestyle='dashed')
fig.colorbar(surf)


plt.show()


# plt.plot(vol[0,:], vol[1,:],vol[2,:], 'o')
# plt.show()

