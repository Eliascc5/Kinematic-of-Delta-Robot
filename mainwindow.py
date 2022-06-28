# -*- coding: utf-8 -*-
"""
Created on Thu Jun 23 17:00:39 2022

@author: EliasC
"""
from libraries import *
from RobotDelta import *
    
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):

    def __init__(self):
    
        super(MainWindow, self).__init__()
        self.setupUi(self)
        
        self.Robot = RobotDelta ()
        
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        self.pushButton.clicked.connect(self.starting)
        
        self.Button_xp.clicked.connect(self.move_xm)
        
    def move_xm(self):
        
        print("aqui")
        
        x,y,z = self.Robot.getCoord()
        
        x = x-0.08
        
        self.Robot.setCoord(x, y, z)
        
        th1,th2,th3 = self.Robot.inverse_Kinematics(x,y,z)
        
        self.Robot.setCoordArt(th1, th2, th3)

        
        triangulo,eslabon1,triangulo_f,eslabon2 = self.Robot.getModel()
        
        self.ax.cla()
        self.ax.set_xlim3d(-0.5, 0.5)
        self.ax.set_ylim3d(-0.5, 0.5)
        self.ax.set_zlim3d(-1.5, 2)
        
        idx = 0 
        
        for j in range(3):
            
            self.ax.plot3D(triangulo[j,idx,:], triangulo[j,idx+1,:], triangulo[j,idx+2,:],'red')
            self.ax.plot3D(eslabon1[j,idx,:], eslabon1[j,idx+1,:], eslabon1[j,idx+2,:],'green')
            self.ax.plot3D(triangulo_f[j,idx,:], triangulo_f[j,idx+1,:], triangulo_f[j,idx+2,:],'blue')
            self.ax.plot3D(eslabon2[j,idx,:], eslabon2[j,idx+1,:], eslabon2[j,idx+2,:],'red')
            
            plt.pause(0.01)
        
        
        
        
        
    def starting(self):
   
        
        ang1=np.arange(-40*DEG_TO_RAD,20*DEG_TO_RAD,0.75*DEG_TO_RAD)
        ang2=np.arange(-10*DEG_TO_RAD,50*DEG_TO_RAD,0.75*DEG_TO_RAD)
        ang3=np.arange(0*DEG_TO_RAD,60*DEG_TO_RAD,0.75*DEG_TO_RAD)
        
        
        i=0
        
        while i <len(ang1):

            x,y,z_neg = self.Robot.forward_Kinematics(ang1[i], ang2[i], ang3[i])
            
            self.Robot.setCoord(x, y, z_neg)
            self.Robot.setCoordArt(ang1[i], ang2[i], ang3[i])
            
            triangulo,eslabon1,triangulo_f,eslabon2 = self.Robot.getModel()
            
            self.ax.cla()
            self.ax.set_xlim3d(-0.5, 0.5)
            self.ax.set_ylim3d(-0.5, 0.5)
            self.ax.set_zlim3d(-1.5, 2)
            

            for j in range(3):
        
                self.ax.plot3D(triangulo[j,0,:], triangulo[j,1,:], triangulo[j,2,:],'red')
                self.ax.plot3D(eslabon1[j,0,:], eslabon1[j,1,:], eslabon1[j,2,:],'green')
                self.ax.plot3D(triangulo_f[j,0,:], triangulo_f[j,1,:], triangulo_f[j,2,:],'blue')
                self.ax.plot3D(eslabon2[j,0,:], eslabon2[j,1,:], eslabon2[j,2,:],'red')
        
        
                plt.pause(0.01)
                
            i = i+1
        
        
