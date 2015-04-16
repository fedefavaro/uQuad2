% Test de Carrot Chase algorithm
clear all
close all
clc
% pi
% pi = 3.14159265358979323846;

% Waypoint actual W(i)
W_act = [2 2];              % W_i = (xi, yi)
% Waypoint siguiente W(i+1)
W_sig = [100 100];			% W_i+1 = (xi+1, yi+1)
% Posicion del UAV
px = 20;
py = -50;
p = [px py];					% p = (x y);
% Heading angle (YAW)
psi = pi/6;

% VTP
s = [0 0];                  % s = [xt' yt'];

%Señal de control
U = 0;
%Ganancia contorl
k = 2;

% Velocidad
v = 0.55;                     % m/s
% Distancia
d = sqrt( ( W_sig(1) - W_act(1) )^2 + ( W_sig(2) - W_act(2) )^2 );
% Tiempo
t = 0.1;                      % s

figure(1)
hold on
grid on
plot(W_act(1), W_act(2), 'ro')
plot(W_sig(1), W_sig(2), 'ro')
plot(p(1), p(2), 'ko')

line = 0:0.01:d;
theta = atan2( W_sig(2) - W_act(2), W_sig(1) - W_act(1));
x = W_act(1) + line*cos(theta);
y = W_act(2) + line*sin(theta);
plot(x,y,'r-')

i = 1;

while px < W_sig(1)

   % Pasos 1 - 6
   psi_d = carrotChase(p, W_act, W_sig);
   
   % Paso 7 - control
   u = k*(psi_d - psi);
   
   %%%%%%%%%%%%%%%%%%%%%%
   % Modelo simplificado
   %%%%%%%%%%%%%%%%%%%%%%
   % dx = v*cos(psi);
   % dy = v*sin(psi);
   % dpsi = u;
   
   % obtengo angulo
   psi = psi + u*t;
   
   % Actualizo posicion en funcion del angulo
   px = px + v*cos(psi);
   py = py + v*sin(psi);
   p = [px py];
   
   plot(px,py,'bo')
   
   i=i+1;
    
end





