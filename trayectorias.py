# -*- coding: utf-8 -*-
"""
Created on Tue May 17 17:15:08 2022

@author: EliasC
"""

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math

RAD_TO_DEF = 180/math.pi

from RobotDelta import RobotDelta

import math
DEG_TO_RAD = math.pi/180

Robot = RobotDelta()

PuntosTime = 10


def trayectoria_pol3(q0=[None],qf=[None], ti=0 ,tf=1):
    
    nroArt = len(q0)
    nroCoef = 4  #Polinomio cubico
    
    coef = np.zeros((nroArt,nroCoef))
    
    t = np.linspace(ti,tf,1000)

    #Matriz de coeficientes del polinomio
    i=0 
    for j in range(nroArt):
            
        coef[j][i] = q0[j]
        coef[j][i+1] = 0
        coef[j][i+2] = (3/pow(tf,2))*(qf[j] - q0[j])
        coef[j][i+3] = (-2/pow(tf,3))*(qf[j] - q0[j])
      
    
    q = [] #Lista las posiciones articulares
    
    qd = [] #Lista de las velocidades articulares
    
    qdd = [] #Lista de las aceleraciones articulares
    
    
    j =0
    
    for i in range(nroArt):
        q.append(coef[i][j] + coef[i][j+1]*t+ coef[i][j+2]*pow(t,2)+coef[i][j+3]*pow(t,3))
        qd.append(coef[i][j+1]+2*coef[i][j+2]*t + 3*coef[i][j+3]*pow(t,2))
        qdd.append(2*coef[i][j+2]+6*coef[i][j+3]*t)
    
    return q, qd, qdd

def trayectoria_pol5(q0=[None],qf=[None], ti=0 ,tf=1):
    
    nroArt = len(q0)
    nroCoef =  6 #Polinomio cubico
    
    coef = np.zeros((nroArt,nroCoef))
    
    t = np.linspace(ti,tf,PuntosTime)

    #Matriz de coeficientes del polinomio
    i=0 
    
    qd0 =[0,0,0]  #Velocidad inicial
    qdf =[1,1,1] #Velocidad final
    qdd0 =[0,0,0] #Aceleracion inicial
    qddf =[0,0,0] #Aceleracion final
    
    for j in range(nroArt):
            
        coef[j][i] = (6/pow(tf,5))*(qf[j]-q0[j])-(3/pow(tf,4))*(qdf[j]+qd0[j])+(1/2*pow(tf,3)*qddf[j])
        coef[j][i+1] = (-15/pow(tf,4))*(qf[j]-q0[j])+(8/pow(tf,3))*qd0[j]+(7/pow(tf,3))*qdf[j]+(1/2*pow(tf,2))*qdd0[j]-(1/pow(tf,2))*qddf[j]
        coef[j][i+2] = (10/pow(tf,3))*(qf[j]-q0[j])-(6/pow(tf,2))*qd0[j]-(4/pow(tf,2))*qdf[j]-(1/pow(tf,1))*qdd0[j]+(1/2*pow(tf,1))*qddf[j]
        coef[j][i+3] = 1/2*qdd0[j]
        coef[j][i+4] = qd0[j]
        coef[j][i+5] = q0[j]
        
      
    
    q = [] #Lista las posiciones articulares
    
    qd = [] #Lista de las velocidades articulares
    
    qdd = [] #Lista de las aceleraciones articulares
    
    
    i = 0
    
    for j in range(nroArt):
        q.append(coef[j][i]*pow(t,5)+coef[j][i+1]*pow(t,4)+coef[j][i+2]*pow(t,3)+coef[j][i+3]*pow(t,2)+coef[j][i+4]*t+coef[j][i+5])
        qd.append(5*coef[j][i]*pow(t,4)+4*coef[j][i+1]*pow(t,3)+3*coef[j][i+2]*pow(t,2)+2*coef[j][i+3]*t+coef[j][i+4])
        qdd.append(20*coef[j][i]*pow(t,3)+12*coef[j][i+1]*pow(t,2)+6*coef[j][i+2]*t+2*coef[j][i+3])
    
    return q, qd, qdd


_q0 = [0,0,-0.5]
_qf = [0.1,0.1,-0.4]

Robot = RobotDelta()

t_init = 0
t_final = 1 #seg

pos,vel,acel = trayectoria_pol5(_q0,_qf,t_init,t_final)


q1=[]
q2=[]
q3=[]

for i in range(len(pos[0][:])):
    
    _q1,_q2,_q3 = Robot.inverse_Kinematics(pos[0][i], pos[1][i], pos[2][i])
    q1.append(_q1*RAD_TO_DEF)
    q2.append(_q2*RAD_TO_DEF)
    q3.append(_q3*RAD_TO_DEF)

t = np.linspace(t_init,t_final,PuntosTime)



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(pos[0],pos[1],pos[2], label='Posicion XYZ')
ax.legend()
plt.show()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(vel[0],vel[1],vel[2], label='Velocidad XYZ')
ax.legend()
plt.show()



plt.figure()
plt.title('Coordenadas articulares')
plt.plot(t,q1, label='tita1')
plt.plot(t,q2, label='tita2')
plt.plot(t,q3, label='tita3')

plt.xlabel('tiempo')
plt.ylabel('Pos angular en grados')
plt.legend()
plt.show()

    
# plt.figure()
# plt.title('Velocidades articulares')
# for i in range(len(q)):
#     plt.plot(t,qd[i], label =f'qd{i}')
# plt.xlabel('tiempo')
# plt.ylabel('Vel angular en grados/s')
# plt.legend()
# plt.show()


# plt.figure()
# plt.title('Aceleraciones articulares')
# for i in range(len(q)):    
#     plt.plot(t,qdd[i], label =f'qdd{i}')
# plt.xlabel('tiempo')
# plt.ylabel('Acel angular en grados/s2')    
# plt.legend()
# plt.show()
    
