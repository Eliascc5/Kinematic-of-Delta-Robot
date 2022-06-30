# -*- coding: utf-8 -*-
"""
Created on Fri Apr 22 15:02:11 2022

@author: EliasC
"""

from libraries import *
from numpy.linalg import inv

class RobotDelta ():
    
    def __init__(self):
        
        #Parametros constructivos
        #Longitudes del robot

        
        self.Sb=0.581        #Base equilateral triangle side
        self.Sp=0.1212       #Platform equilateral triangle side
        
        self.L = 0.4         #Upper legs length
        self.l = 0.6         #Lower legs parallelogram length
        self.h = 0.131       #Lower legs parallelogram width
           
        self.wb=(math.sqrt(3)/6)*self.Sb            #Planar distance from {0} to near base side
        self.ub=(math.sqrt(3)/3)*self.Sb            #Planar distance from {0} to a base vertex 
        self.wp=(math.sqrt(3)/6)*self.Sp            #Planar distance from {P} to near platform side
        self.up=(math.sqrt(3)/3)*self.Sp            #Planar distance from {P} to a platform vertex
        
        self.a = self.wb-self.up
        self.b = self.Sp/2-(math.sqrt(3)/2)*self.wb
        self.c = self.wp-0.5*self.wb
        
        #Coordenadas de la posocion de los motores
        self.BB = np.array([[0,-self.wb,0],[(math.sqrt(3)/2)*self.wb,1/2*self.wb,0],[-(math.sqrt(3)/2)*self.wb,1/2*self.wb,0]])
        
        #Vertices of the base equilateral triangle
        self.verticesTriangleBase = np.array([[self.Sb/2,-self.wb,0],[0,self.ub,0], [-self.Sb/2, -self.wb, 0]])
        
        

        self.P1=np.array([0,-self.up,0])
        self.P2=np.array([self.Sp/2,self.wp,0])
        self.P3=np.array([-self.Sp/2,self.wp,0])
            
        #Posicion inicial 
        self.x=0
        self.y=0
        self.z=-0.334658034417224
        
        self.theta1,self.theta2,self.theta3 = self.inverse_Kinematics(self.x, self.y, self.z)
        
        ###########################################
        dim = 50
        
        self.triangulo = np.zeros((3,3,dim))
        self.triangulo_f = np.zeros((3,3,dim))
        self.eslabon1 = np.zeros((3,3,dim))
        self.eslabon2 = np.zeros((3,3,dim))
        
    def setCoord(self,_x,_y,_z):
        self.x,self.y,self.z=_x,_y,_z
        
    def getCoord(self):
        return self.x,self.y,self.z
    

    def setCoordArt(self,_theta1,_theta2,_theta3):
        
        self.theta1,self.theta2,self.theta3 = _theta1,_theta2,_theta3
        print("los anguloss", self.theta1,self.theta2,self.theta3)
        
    def getCoordArt(self):
        return  self.theta1,self.theta2,self.theta3       
        
    def forward_Kinematics(self,theta1, theta2,theta3):

        
        A1=np.array([0,-self.wb-self.L*math.cos(theta1),-self.L*math.sin(theta1)])
        A2=np.array([(math.sqrt(3)/2)*(self.wb+self.L*math.cos(theta2)),0.5*(self.wb+self.L*math.cos(theta2)),-self.L*math.sin(theta2)])
        A3=np.array([-(math.sqrt(3)/2)*(self.wb+self.L*math.cos(theta3)),0.5*(self.wb+self.L*math.cos(theta3)),-self.L*math.sin(theta3)])
        
        #Desplazamiento del centro de las tres esferas. Para lograr la interseccion un punto
        
        A1v = A1-self.P1
        A2v = A2-self.P2
        A3v = A3-self.P3
        
        #Centro de las tres esferas
        
        x1 = round(A1v[0],4)
        x2 = round(A2v[0],4)
        x3 = round(A3v[0],4)
        
        y1 = round(A1v[1],4)
        y2 = round(A2v[1],4)
        y3 = round(A3v[1],4)
        
        z1 = round(A1v[2],4)
        z2 = round(A2v[2],4)
        z3 = round(A3v[2],4)
        
        # print(x1)
        # print(y1)
        # print(z1)
        
                
        # print(x2)
        # print(y2)
        # print(z2)
        
                
        # print(x3)
        # print(y3)
        # print(z3)
        
        #Aca salvamos un sigularidad del algoritmo que NO representa una restriccion de movimiento 
        #Cuando las tres esferas tiene la misma coordenada z
        
        if z1==z2 or z2==z3 or z1==z3:
        
            print('dos o tres Z iguales')
            
       
            a = 2*(x3-x1)
            b = 2*(y3-y1)
            c = pow(self.l,2)- pow(self.l,2)-pow(x1,2)-pow(y1,2)+pow(x3,2)+pow(y3,2)
            d = 2*(x3-x2)
            e = 2*(y3-y2)
            f = pow(self.l,2)- pow(self.l,2)-pow(x2,2)-pow(y2,2)+pow(x3,2)+pow(y3,2)
               
            #The unique solution for x and y :
            
            x = (c*e-b*f)/(a*e-b*d)
            y = (a*f-c*d)/(a*e-b*d)
            
            print('Valor de x para cuando z son iguales: ', x)
            print('Valor de y para cuando z son iguales: ', y)
            
            #Calculamos los dos valores de z
            
            if z1 == z2: zn=z1 
            else: zn=z3
            
            A = 1
            B = -2*zn
            C = pow(zn,2)-pow(self.l,2)+pow(x,2)+pow(y,2)+pow(x1,2)+pow(y1,2)-2*x*x1-2*y*y1
            
            z_positive=(-B+math.sqrt((pow(B,2)-4*A*C)))/(2*A)
            z_negative=(-B-math.sqrt((pow(B,2)-4*A*C)))/(2*A)
            z_=0
                     
            if z_positive>=0:
                z_=z_negative
                print('z :', z_negative)
            elif z_negative >=0:
                print('z :', z_positive) 
                z_=z_positive
            else:
                print("Error eje z")
                
            return x,y,z_

        else:
            #Algortimo : Punto de interseccion de 3 esferas  
            
            print('z distintas') 
            print("--------------")
            
            a11=2*(x3-x1)
            a12=2*(y3-y1)
            a13=2*(z3-z1)
            
            a21=2*(x3-x2)
            a22=2*(y3-y2)
            a23=2*(z3-z2)
            
            b1=-pow(x1,2)-pow(y1,2)-pow(z1,2)+pow(x3,2)+pow(y3,2)+pow(z3,2)
            b2=-pow(x2,2)-pow(y2,2)-pow(z2,2)+pow(x3,2)+pow(y3,2)+pow(z3,2)
            
            a1=(a11/a13)-(a21/a23)
            a2=(a12/a13)-(a22/a23)
            a3=(b2/a23)-(b1/a13)
            a4=-a2/a1
            a5=-a3/a1
            a6=((-a21*a4)-a22)/a23
            a7=(b2-(a21*a5))/a23
                        
            a=pow(a4,2)+1+pow(a6,2)
            b=(2*a4)*(a5-x1)-(2*y1) +(2*a6)*(a7-z1)
            c=a5*(a5-2*x1)+a7*(a7-2*z1)+pow(x1,2)+pow(y1,2)+pow(z1,2)-pow(self.l,2)
            
            if z1<z2 and z1<z3 and z3<z2: #puntos que estan en el cuadrante  (+y,+x)
                print("caso1")
                
                y_positive=(-b+math.sqrt((pow(b,2)-4*a*c)))/(2*a)
                x_positive=a4*y_positive+a5
                
                print('x_positive',x_positive)
                print('y_positive',y_positive)
                
                z_positive=a6*y_positive+a7
                
                print('z:',z_positive)
                
                return x_positive,y_positive,z_positive
                
            elif z1<z2 and z1<z3 and z2<z3: #puntos que estan en el cuadrante  (+y,-x)
                print("caso2")
                y_positive=(-b+math.sqrt((pow(b,2)-4*a*c)))/(2*a)
                x_positive=a4*y_positive+a5
                
                print('x_positive',x_positive)
                print('y_positive',y_positive)
                
                z_positive=a6*y_positive+a7
                
                print('z:',z_positive)
                
                return x_positive,y_positive,z_positive

                
            else:
                print("caso4")
                y_negative=(-b-math.sqrt((pow(b,2)-4*a*c)))/(2*a)
                x_negative=a4*y_negative+a5
                
                print('x_negative',x_negative)
                print('y_negative',y_negative)  
            
                z_negative=a6*y_negative+a7
                
                print('z:',z_negative)
                
                return x_negative,y_negative,z_negative
    
    def inverse_Kinematics(self,x,y,z):
        
        
        E1=2*self.L*(y+self.a)
        F1=2*z*self.L
        G1=pow(x,2)+pow(y,2)+pow(z,2)+pow(self.a,2)+pow(self.L,2)+2*y*self.a-pow(self.l,2)
        
        E2=-self.L*(math.sqrt(3)*(x+self.b)+y+self.c)
        F2=2*z*self.L
        G2=pow(x,2)+pow(y,2)+pow(z,2)+pow(self.b,2)+pow(self.c,2)+pow(self.L,2)+2*(x*self.b+y*self.c)-pow(self.l,2)
        
        E3=self.L*(math.sqrt(3)*(x-self.b)-y-self.c)
        F3=2*z*self.L
        G3=pow(x,2)+pow(y,2)+pow(z,2)+pow(self.b,2)+pow(self.c,2)+pow(self.L,2)+2*(-x*self.b + y*self.c)-pow(self.l,2)
        
        t1_positive=(-F1+math.sqrt(abs(pow(E1,2)+pow(F1,2)-pow(G1,2))))/(G1-E1)
        t1_negative=(-F1-math.sqrt(abs(pow(E1,2)+pow(F1,2)-pow(G1,2))))/(G1-E1)
        
        t2_positive= (-F2+math.sqrt(abs(pow(E2,2)+pow(F2,2)-pow(G2,2))))/(G2-E2)
        t2_negative=(-F2-math.sqrt(abs(pow(E2,2)+pow(F2,2)-pow(G2,2))))/(G2-E2)
        
        t3_positive=(-F3+math.sqrt(abs(pow(E3,2)+pow(F3,2)-pow(G3,2))))/(G3-E3)
        t3_negative=(-F3-math.sqrt(abs(pow(E3,2)+pow(F3,2)-pow(G3,2))))/(G3-E3)
     

        theta1 =(2*math.atan(t1_negative))
        theta2 =(2*math.atan(t2_negative))
        theta3 =(2*math.atan(t3_negative))
        
        
        # Analysis of the multiple solutions
        
        # theta1_p = 2*math.atan(t1_positive)
        # theta2_p = 2*math.atan(t2_positive)
        # theta3_p = 2*math.atan(t3_positive)

        # print("theta1 positive:",theta1_p * RAD_TO_DEF)  #Non valid
        # print("theta1 :",theta1_n * RAD_TO_DEF)          #kinked out solution
        # print("--------------")
        # print("theta2 positive:",theta2_p * RAD_TO_DEF)  #Non valid
        # print("theta2 :",theta2_n * RAD_TO_DEF)          #kinked out solution
        # print("--------------")
        # print("theta3 positive:",theta3_p * RAD_TO_DEF)  #Non valid
        # print("theta3 :",theta3_n * RAD_TO_DEF)          #kinked out solution   

        return theta1 ,theta2 ,theta3 
    
    def inverseJacobian(self,x,y,z,velx, vely,velz):
        
        vel = np.array([velx,vely,velz])
        
        th1,th2,th3 = self.inverse_Kinematics(x, y, z)
        
        print("en jacobian",th1)
      
        A=np.array([[x, y+self.a+self.L*math.cos(th1),z+self.L*math.sin(th1)],
        [2*(x+self.b)-math.sqrt(3)*self.L*math.cos(th2), 2*(y+self.c)-self.L*math.cos(th2), 2*(z+self.L*math.sin(th2))],
        [2*(x-self.b)+math.sqrt(3)*self.L*math.cos(th3), 2*(y+self.c)-self.L*math.cos(th3), 2*(z+self.L*math.sin(th3))]])
     
       
        b11 = self.L*((y+self.a)*math.sin(th1)-z*math.cos(th1))
        b22 =-self.L*((math.sqrt(3)*(x+self.b)+y+self.c)*math.sin(th2)+2*z*math.cos(th2))
        b33 = self.L*((math.sqrt(3)*(x-self.b)-y-self.c)*math.sin(th3)-2*z*math.cos(th3))
        
        B=np.array([[b11,0,0],[0,b22,0],[0,0,b33]])
        
        
        omegas = (inv(B)*A)*vel
        
        return np.diagonal(omegas)

        
    def getModel(self):
        
        
        Lin = np.array([[0,-self.L*(math.cos(self.theta1)),-self.L*math.sin(self.theta1)],
                        [(math.sqrt(3)/2)*self.L*(math.cos(self.theta2)),0.5*self.L*(math.cos(self.theta2)),-self.L*math.sin(self.theta2)],
                        [-(math.sqrt(3)/2)*self.L*(math.cos(self.theta3)),0.5*self.L*(math.cos(self.theta3)),-self.L*math.sin(self.theta3)]])

        Pp = np.array([[0+self.x,-self.up+self.y,0+self.z],[self.Sp/2+self.x,self.wp+self.y,0+self.z],[-self.Sp/2+self.x,self.wp+self.y,0+self.z]])     

        #Plataforma fija y plat. movil

        for i in range(3):
            for j in range(3):
                if i!=2:
                    self.triangulo[i][j] = np.linspace(self.verticesTriangleBase[i][j],self.verticesTriangleBase[i+1][j])     
                    self.triangulo_f[i][j] = np.linspace(Pp[i][j],Pp[i+1][j])
                else:                        
                    self.triangulo[i][j] = np.linspace(self.verticesTriangleBase[i][j],self.verticesTriangleBase[i-2][j])
                    self.triangulo_f[i][j] = np.linspace(Pp[i][j],Pp[i-2][j])
    
        #Eslabon 1 y Eslabon 2

        for i in range(3):
            for j in range(3):
                self.eslabon1[i][j]=np.linspace(self.BB[i][j],Lin[i][j])
                self.eslabon2[i][j]=np.linspace(Lin[i][j],Pp[i][j])


        return self.triangulo,self.eslabon1,self.triangulo_f,self.eslabon2

# Robot = RobotDelta()



# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')


# ang1=np.arange(-40*DEG_TO_RAD,20*DEG_TO_RAD,0.75*DEG_TO_RAD)
# ang2=np.arange(-30*DEG_TO_RAD,30*DEG_TO_RAD,0.75*DEG_TO_RAD)
# ang3=np.arange(0*DEG_TO_RAD,60*DEG_TO_RAD,0.75*DEG_TO_RAD)


# i=0

# while i <len(ang1):

#     x,y,z_neg = Robot.forward_Kinematics(ang1[i], ang2[i], ang3[i])
    
#     Robot.setCoord(x, y, z_neg)
#     Robot.setCoordArt(ang1[i], ang2[i], ang3[i])
    
#     triangulo,eslabon1,triangulo_f,eslabon2 = Robot.getModel()
    
#     ax.cla()
#     ax.set_xlim3d(-0.5, 0.5)
#     ax.set_ylim3d(-0.5, 0.5)
#     ax.set_zlim3d(-1.5, 2)
    


#     idx=0
    
#     ax.plot3D(triangulo[0,idx,:], triangulo[0,idx+1,:], triangulo[0,idx+2,:],'red')
#     ax.plot3D(triangulo[1,idx,:], triangulo[1,idx+1,:], triangulo[1,idx+2,:],'red')
#     ax.plot3D(triangulo[2,idx,:], triangulo[2,idx+1,:], triangulo[2,idx+2,:],'red')
  
#     for j in range(3):

#         # ax.plot3D(triangulo[j,idx,:], triangulo[j,idx+1,:], triangulo[j,idx+2,:],'red')
#         ax.plot3D(eslabon1[j,idx,:], eslabon1[j,idx+1,:], eslabon1[j,idx+2,:],'green')
#         ax.plot3D(triangulo_f[j,idx,:], triangulo_f[j,idx+1,:], triangulo_f[j,idx+2,:],'blue')
#         ax.plot3D(eslabon2[j,idx,:], eslabon2[j,idx+1,:], eslabon2[j,idx+2,:],'red')


#         plt.pause(0.001)
        
#     i = i+1


#####################################################################


Robot = RobotDelta()



# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# # x=np.linspace(-0.1,0.2,50)
# # y=np.linspace(-0.1,0.2,50)
# # z=np.linspace(-0.7,-0.7,50)

# x=np.linspace(0,0,50)
# y=np.linspace(0,0,50)
# z=np.linspace(-0.7,-0.2,50)
# i=0

# while i <len(x):

#     th1,th2,th3 = Robot.inverse_Kinematics(x[i], y[i], z[i])
#     Robot.setCoordArt(th1, th2, th3)
#     Robot.setCoord(x[i], y[i], z[i])

    
    
#     triangulo,eslabon1,triangulo_f,eslabon2 = Robot.getModel()
    
#     ax.cla()
#     ax.set_xlim3d(-0.5, 0.5)
#     ax.set_ylim3d(-0.5, 0.5)
#     ax.set_zlim3d(-1.5, 2)
    


#     idx=0
    
#     for j in range(3):

#         ax.plot3D(triangulo[j,idx,:], triangulo[j,idx+1,:], triangulo[j,idx+2,:],'red')
#         ax.plot3D(eslabon1[j,idx,:], eslabon1[j,idx+1,:], eslabon1[j,idx+2,:],'green')
#         ax.plot3D(triangulo_f[j,idx,:], triangulo_f[j,idx+1,:], triangulo_f[j,idx+2,:],'blue')
#         ax.plot3D(eslabon2[j,idx,:], eslabon2[j,idx+1,:], eslabon2[j,idx+2,:],'red')


#         plt.pause(0.1)
        
#     i = i+1
