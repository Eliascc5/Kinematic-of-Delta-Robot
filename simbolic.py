# -*- coding: utf-8 -*-
"""
Created on Mon May  9 20:12:34 2022

@author: EliasC
"""

from sympy import *
import numpy as np
import math 


tita1, tita1d, tita2, tita2d, tita3, tita3d, x, y, z, xd, yd, zd, a, b, c, L, Lc1, Lc2, Lc3, l_2, m1, m2, m3, I1, I2, g = symbols('tita1 tita1d tita2 tita2d tita3 tita3d x y z xd yd zd a b c L Lc1 Lc2 Lc3 l_2 m1 m2 m3 I1 I2 g')


# Dinámica inversa


#  posición centro de gravedad articulación 1
#  ver si está en la mitad o no, hay que cambiar el 1/2

Lc1 = 1/2 * np.array([0,- L * cos(tita1),-L * sin(tita1)])
      
Lc2= 1/2* np.array([math.sqrt(3)/2*L*cos(tita2), 1/2 * L *cos(tita2),- L * sin(tita2)])

Lc3= 1/2* np.array([-math.sqrt(3)/2*L*cos(tita3), 1/2 * L *cos(tita3),- L * sin(tita3)])


# Lc3= 1/2* [-sqrt(3)/2*L*cos(tita3);
#             1/2 * L *cos(tita3);
#             - L * sin(tita3)];
 
# %% velocidad del eslabón 1 en cada articulación
# L1d = [0;
#      +L * tita1d * sin(tita1);
#      -L * tita1d * cos(tita1)];
 
# L2d = [-sqrt(3)/2 * L * sin(tita2) * tita2d;
#     -1/2 * L * sin(tita2) * tita2d;
#     -L * cos(tita2) * tita2d];

# L3d = [sqrt(3)/2 * L * sin(tita3) * tita3d;
#     -1/2 * L * sin(tita3) * tita3d;
#     -L * cos(tita3) * tita3d];
# %% Posición articulación 2
# l1 = [x;
#     y + L*cos(tita1) + a;
#     z + L*sin(tita1)];

# l2 = [x - sqrt(3)/2 * L * cos(tita2) + b;
#     y - 1/2 * L * cos(tita2) + c;
#     z + L * sin(tita2)];

# l3 = [x + sqrt(3)/2 * L * cos(tita3) - b;
#     y - 1/2 * L * cos(tita3) + c;
#     z + L * sin(tita3)];

# %% veocidad efector final
# p = [x;y;z];

# pd = [xd;
#       yd;
#       zd];
 
# aux1 = pd - L1d  ;
# aux2 = pd - L2d;
# aux3 = pd - L3d;

# numerador1 = cross(l1,aux1);
# w21 = numerador1/l_2;
# numerador2 = cross(l2,aux2);
# w22 = numerador2/l_2;
# numerador3 = cross(l3,aux3);
# w23 = numerador3/l_2;

# %% velocidad en el CENTRO de MASA
# vc21 = pd - cross(w21,l1)/2
# vc22 = pd - cross(w22,l2)/2
# vc23 = pd - cross(w23,l3)/2

# %% Energía cinética
# K11 = 1/2 * m1 * tita1d^2 * Lc1.^2 + 1/2 * I1 * tita1d^2;
# K12 = 1/2 * m1 * tita2d^2 * Lc2.^2 + 1/2 * I1 * tita2d^2;
# K13 = 1/2 * m1 * tita3d^2 * Lc3.^2 + 1/2 * I1 * tita3d^2;

# K21 = 1/2 * m2 * vc21.^2 + 1/2 * I2 * w21.^2;
# K22 = 1/2 * m2 * vc22.^2 + 1/2 * I2 * w22.^2;
# K23 = 1/2 * m2 * vc23.^2 + 1/2 * I2 * w23.^2;

# Kp = 1/2 * m3 * pd.^2;

# K =  Kp + K11 + K12 + K13 + K21 + K22 + K23


# %% Energía potencial

# U11 = m1 * g * Lc1(3);  
# U12 = m1 * g * Lc2(3);  
# U13 = m1 * g * Lc3(3);  

# U21 = m2 * g * 1/2 * (p(3) + L * sin(tita1));  %expresar bien L
# U22 = m2 * g * 1/2 * (p(3) + L * sin(tita3));  %expresar bien L
# U23 = m2 * g * 1/2 * (p(3) + L * sin(tita3));  %expresar bien L

# Up = m3 * g * p(3);

# U  =  Up + U11 + U12 + U13 + U21 + U22 + U23;

# %% Lagrangiano

# L = K-U;
