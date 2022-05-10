%% Dinámica inversa

% Sb=0.567
% Sp=0.076
%
% L = 0.524
% l_ = 1.244
% h = 0.131

syms t
syms tita1(t) tita1d(t) tita2(t) tita2d(t) tita3(t) tita3d(t) x(t) y(t) z(t) xd(t) yd(t) zd(t) a b c L  l_2 m1 m2 m3 I1 I2 g;

%% posición centro de gravedad articulación 1
% ver si está en la mitad o no, hay que cambiar el 1/2
Lc1 = 1/2* [0;
            - L * cos(tita1);
             -L * sin(tita1)];

Lc2= 1/2* [sqrt(3)/2*L*cos(tita2);
            1/2 * L *cos(tita2);
            - L * sin(tita2)];

Lc3= 1/2* [-sqrt(3)/2*L*cos(tita3);
            1/2 * L *cos(tita3);
            - L * sin(tita3)];

%% velocidad del eslabón 1 en cada articulación
L1d = [0;
     +L * tita1d * sin(tita1);
     -L * tita1d * cos(tita1)];

L2d = [-sqrt(3)/2 * L * sin(tita2) * tita2d;
    -1/2 * L * sin(tita2) * tita2d;
    -L * cos(tita2) * tita2d];

L3d = [sqrt(3)/2 * L * sin(tita3) * tita3d;
    -1/2 * L * sin(tita3) * tita3d;
    -L * cos(tita3) * tita3d];
%% Posición articulación 2
l1 = [x;
    y + L*cos(tita1) + a;
    z + L*sin(tita1)];

l2 = [x - sqrt(3)/2 * L * cos(tita2) + b;
    y - 1/2 * L * cos(tita2) + c;
    z + L * sin(tita2)];

l3 = [x + sqrt(3)/2 * L * cos(tita3) - b;
    y - 1/2 * L * cos(tita3) + c;
    z + L * sin(tita3)];

%% veocidad efector final
p = [x;
     y;
     z];

pd = [xd;
      yd;
      zd];

aux1 = pd - L1d  ;
aux2 = pd - L2d;
aux3 = pd - L3d;

numerador1 = cross(l1,aux1);
w21 = numerador1/l_2;
numerador2 = cross(l2,aux2);
w22 = numerador2/l_2;
numerador3 = cross(l3,aux3);
w23 = numerador3/l_2;

%% velocidad en el CENTRO de MASA
vc21 = pd - cross(w21,l1)/2;
vc22 = pd - cross(w22,l2)/2;
vc23 = pd - cross(w23,l3)/2;

%% Energía cinética
K11 = 1/2 * (m1 * Lc1.^2 + I1) * tita1d^2;
K12 = 1/2 * (m1 * Lc2.^2 + I1) * tita2d^2;
K13 = 1/2 * (m1 * Lc3.^2 + I1) * tita3d^2;

K21 = 1/2 * (m2 * vc21.^2 + I2 * w21.^2);
K22 = 1/2 * (m2 * vc22.^2 + I2 * w22.^2);
K23 = 1/2 * (m2 * vc23.^2 + I2 * w23.^2);

Kp = 1/2 * m3 * pd.^2;

K =  Kp + K11 + K12 + K13 + K21 + K22 + K23;


%% Energía potencial

U11 = m1 * g * Lc1(3);
U12 = m1 * g * Lc2(3);
U13 = m1 * g * Lc3(3);

U21 = m2 * g * 1/2 * (p(3) + L * sin(tita1));  %expresar bien L
U22 = m2 * g * 1/2 * (p(3) + L * sin(tita3));  %expresar bien L
U23 = m2 * g * 1/2 * (p(3) + L * sin(tita3));  %expresar bien L

Up = m3 * g * p(3);

U  =  Up + U11 + U12 + U13 + U21 + U22 + U23;

%% Lagrangiano
%Coordenadas generalizadas
% Px,Py,Pz, tita1, tita2, tita3

L = K-U;

dLdPx = diff(L,x);
dLdPy = diff(L,y);
dLdPz = diff(L,z);

dLdTita1 = diff(L,tita1);
dLdTita2 = diff(L,tita2);
dLdTita3 = diff(L,tita3);

dLdTita1d = diff(L,tita1d);
dLdTita2d = diff(L,tita2d);
dLdTita3d = diff(L,tita3d);

dLdPxd = diff(L,xd);
dLdPyd = diff(L,yd);
dLdPzd = diff(L,zd);

dtdLdPxd = diff(dLdPxd,t);
dtdLdPyd = diff(dLdPyd,t);
dtdLdPzd = diff(dLdPzd,t);

dtdLdTita1d = diff(dLdTita1d,t);
dtdLdTita2d = diff(dLdTita2d,t);
dtdLdTita3d = diff(dLdTita3d,t);

%Ecuaciones de restriccion

R1 =  2*L*(y(t)+a)*cos(tita1(t)) + 2*z(t)*L*sin(tita1(t))+ x(t)^2 + y(t)^2 + z(t)^2 + a^2 + L^2 + 2*y(t)*a - l_2 ;
R2 = -L*(sqrt(3)*(x(t)+b) + y(t) + c)*cos(tita2(t)) + 2*z(t)*L*sin(tita2(t))+ x(t)^2 + y(t)^2 + z(t)^2 + b^2 + c^2 + L^2 + 2*x(t)*b + 2*y(t)*c - l_2;
R3 = L*(sqrt(3)*(x(t)-b) - y(t) - c)*cos(tita3(t)) + 2*z(t)*L*sin(tita3(t))+ x(t)^2 + y(t)^2 + z(t)^2 + b^2 + c^2 + L^2 - 2*x(t)*b + 2*y(t)*c - l_2;

%Derivamos cada una de las ecuaciones de restriccion respecto de las 6 coordenadas generalizadas

dR1dPx = diff(R1,x);
dR1dPy = diff(R1,y);
dR1dPz = diff(R1,z);

dR2dPx = diff(R2,x);
dR2dPy = diff(R2,y);
dR2dPz = diff(R2,z);

dR3dPx = diff(R3,x);
dR3dPy = diff(R3,y);
dR3dPz = diff(R3,z);

dR1dTita1 = diff(R1,tita1);
dR1dTita1 = diff(R1,tita2); % = 0
dR1dTita1 = diff(R1,tita3); % = 0

dR2dTita2 = diff(R2,tita1); % = 0
dR2dTita2 = diff(R2,tita2);
dR2dTita2 = diff(R2,tita3); % = 0

dR3dTita3 = diff(R3,tita1); % = 0
dR3dTita3 = diff(R3,tita2); % = 0
dR3dTita3 = diff(R3,tita3);

%Obtencion de los mmultiplicadores de lagrange
%se calculan para un estado de movientos dado 

%Pierna 1 

lambda1x = (dtdLdPxd - dLdPx)./ dR1dPx ;

lambda1y = (dtdLdPyd - dLdPy)./ dR1dPy ;

lambda1z = (dtdLdPzd - dLdPz)./ dR1dPz ;

%Pierna 2

lambda2x = (dtdLdPxd - dLdPx)./ dR2dPx ;

lambda2y = (dtdLdPyd - dLdPy)./ dR2dPy ;

lambda2z = (dtdLdPzd - dLdPz)./ dR2dPz ;


%Pierna 3 

lambda3x = (dtdLdPxd - dLdPx)./ dR3dPx ;

lambda3y = (dtdLdPyd - dLdPy)./ dR3dPy ;

lambda3z = (dtdLdPzd - dLdPz)./ dR3dPz ;


%Ecuaciones de torques (Revisar los lambda) evaluar los valores

torque1 = dLdTita1d - dLdTita1 - lambda1 * dR1dTita1 

torque2 = dLdTita2d - dLdTita2 - lambda2 * dR2dTita2 

torque3 = dLdTita3d - dLdTita3 - lambda3 * dR3dTita3
