close all
Kp = 3.5;
Td = 0.4;
T = 0.41;

% transferencias
s = tf('s');
C = (1+Td*s);%/(1+0.005*s);
H = 1/(1+T*s);
I = 1/s;

%H2 = 0.4/(0.12*s^2+0.25*s+0.4);

% lugar geometrico
figure
axis equal
rlocus(C*H*I, [0:0.01:20])

% respuesta escalon y
 Gcl = feedback(Kp*C*H*I,1);
 figure
 step(Gcl)

% respuesta escalon u
% U = Kp*C/(1+Kp*C*I*H);
% figure
% step(U)

% respuesta escalon u
%U = Kp*C*H2*I/(1+Kp*C*I*H2);
%figure
%step(U,[0:0.01:10])

% figure
% step(H)