# -*- coding: utf-8 -*-
"""
@author: correa elias

Reference: 
R.L. Williams II, “The Delta Parallel Robot: Kinematics Solutions”, Internet Publication,
www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf, January 2016.

"""
import numpy as np
import matplotlib.pyplot as plt
import math

#Variables globales
RAD_TO_DEF = 180/math.pi
DEG_TO_RAD = math.pi/180


class RobotDelta ():
    
    def __init__(self):
        
        #Parametros costructivod
        
        #Longitudes del robot comercial : ABB FlexPicker IRB 360-1/1600
 
        self.Sb=0.567            #Base equilateral triangle side
        self.Sp=0.076            #Platform equilateral triangle side
        
        self.L = 0.524            #Upper legs length
        self.l = 1.244            #Lower legs parallelogram length
        self.h = 0.131            #Lower legs parallelogram width
        
        self.wb=(math.sqrt(3)/6)*self.Sb            #Planar distance from {0} to near base side
        self.ub=(math.sqrt(3)/3)*self.Sb            #Planar distance from {0} to a base vertex 
        self.wp=(math.sqrt(3)/6)*self.Sp            #Planar distance from {P} to near platform side
        self.up=(math.sqrt(3)/3)*self.Sp            #Planar distance from {P} to a platform vertex
        
        
        self.B1=np.array([0,-self.wb,0])
        self.B2=np.array([(math.sqrt(3)/2)*self.wb,0.5*self.wb,0])
        self.B3=np.array([(-math.sqrt(3)/2)*self.wb,0.5*self.wb,0])

        self.P1=np.array([0,-self.up,0])
        self.P2=np.array([self.Sp/2,self.wp,0])
        self.P3=np.array([-self.Sp/2,self.wp,0])
            
    
        self.a = self.wb-self.up
        self.b = self.Sp/2-(math.sqrt(3)/2)*self.wb
        self.c = self.wp-0.5*self.wb
        
    def forward_Kinematics(self,theta1, theta2,theta3):
        
        L1=np.array([0,-self.L*math.cos(theta1),-self.L*math.sin(theta1)])
        L2=np.array([(math.sqrt(3)/2)*self.L*math.cos(theta2),0.5*self.L*math.cos(theta2),-self.L*math.sin(theta2)])
        L3=np.array([-(math.sqrt(3)/2)*self.L*math.cos(theta3),0.5*self.L*math.cos(theta3),-self.L*math.sin(theta3)])
        
        
        A1=np.array([0,-self.wb-self.L*math.cos(theta1),-self.L*math.sin(theta1)])
        A2=np.array([(math.sqrt(3)/2)*(self.wb+self.L*math.cos(theta2)),0.5*(self.wb+self.L*math.cos(theta2)),-self.L*math.sin(theta2)])
        A3=np.array([-(math.sqrt(3)/2)*(self.wb+self.L*math.cos(theta3)),0.5*(self.wb+self.L*math.cos(theta3)),-self.L*math.sin(theta3)])
        
        A1v = A1-self.P1
        A2v = A2-self.P2
        A3v = A3-self.P3
        
        x1 = A1v[0]
        x2 = A2v[0]
        x3 = A3v[0]
        
        y1 = A1v[1]
        y2 = A2v[1] 
        y3 = A3v[1]
        
        z1 = A1v[2]
        z2 = A2v[2]
        z3 = A3v[2]
        
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
                     
            print('z positivo (INVALIDO) :', z_positive)
            print('z negativo (VALIDO) :', z_negative)
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
            
            y_positive=(-b+math.sqrt((pow(b,2)-4*a*c)))/(2*a)
            y_negative=(-b-math.sqrt((pow(b,2)-4*a*c)))/(2*a)
            
            x_positive=a4*y_positive+a5
            x_negative=a4*y_negative+a5
            
            z_positive=a6*y_positive+a7
            z_negative=a6*y_negative+a7
            
            
            print('y_positive',y_positive)
            print('y_negative',y_negative)
            print("--------------")
            print('x_positive',x_positive)
            print('x_negative',x_negative)
            print("--------------")
            print('z_positive',z_positive)
            print('z_negative',z_negative)
    
    def inverse_Kinematics(self,x,y,z):
        
     
        cord=np.matrix([[x],[y],[z]])     #Just to test
        
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

        #TO DO:  Analysis of the multiple solutions  
        #If you want to test the forward kinematic model, you have to comment these three lines
        
        theta1_p=2*math.atan(t1_positive)
        theta1 = theta1_n=(2*math.atan(t1_negative))
        
        theta2_p = 2*math.atan(t2_positive)
        theta2 = theta2_n=(2*math.atan(t2_negative))
        
        theta3_p=2*math.atan(t3_positive)
        theta3 = theta3_n=(2*math.atan(t3_negative))
        

                
        print("theta1 positive:",theta1_p * RAD_TO_DEF)
        print("theta1 negative:",theta1_n * RAD_TO_DEF)     #kinked out solution
        print("--------------")
        print("theta2 positive:",theta2_p * RAD_TO_DEF)
        print("theta2 negative:",theta2_n * RAD_TO_DEF)     #kinked out solution
        print("--------------")
        print("theta3 positive:",theta3_p * RAD_TO_DEF)
        print("theta3 negative:",theta3_n * RAD_TO_DEF)     #kinked out solution   



if __name__ == '__main__':
    
    Robot=RobotDelta()
    
    #Put it in radians
    theta1 = 0*DEG_TO_RAD
    theta2 = 10*DEG_TO_RAD
    theta3 = 20*DEG_TO_RAD
    
    # End effector coordinates
    
    # x=0.0
    # y=0.0
    # z=-1.5138166732618923
        
    #Modify Z within the range [-0.9; -1.6]  
    #At maximum depth the radius (x, y) is 0.2 [m]
    
    
    Robot.forward_Kinematics(theta1, theta2, theta3)    
    # Robot.inverse_Kinematics(x, y, z)