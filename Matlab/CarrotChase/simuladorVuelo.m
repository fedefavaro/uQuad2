function [ p_fin, psi_fin ] = simuladorVuelo(p, psi, W_act, W_sig)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%W_act = [W_act_x W_act_y];
%W_sig = [W_sig_x W_sig_y];
p
%Señal de control
U = 0;
%Ganancia contorl
k =4;
% Velocidad
v = 0.55;                     % m/s
% Distancia
d = sqrt( ( W_sig(1) - W_act(1) )^2 + ( W_sig(2) - W_act(2) )^2 );
% Tiempo
t = 0.1;   % seg

plot(W_act(1), W_act(2), 'ro')
plot(W_sig(1), W_sig(2), 'ro')
plot(p(1), p(2), 'ko')

line = 0:0.01:d;
theta = atan2( W_sig(2) - W_act(2), W_sig(1) - W_act(1));

%if(theta<0)
%    theta = 2*pi + theta
%end

x = W_act(1) + line*cos(theta);
y = W_act(2) + line*sin(theta);
plot(x,y,'r-')

i = 1;

Tau = 0.01*norm(W_sig - W_act)

while norm(W_sig - p) > Tau
   % Pasos 1 - 6
   psi_d = carrotChase(p, W_act, W_sig)
   
   % Paso 7 - control
   if(psi<0)
      u = k*(psi_d + psi);
      %psi = psi - u*t;
   else
      u = k*(psi_d - psi);
      %psi = psi + u*t;
   end
   
   %%%%%%%%%%%%%%%%%%%%%%
   % Modelo simplificado
   %%%%%%%%%%%%%%%%%%%%%%
   % dx = v*cos(psi);
   % dy = v*sin(psi);
   % dpsi = u;
   
   % obtengo angulo
   %psi = psi + u*t
   
   % Actualizo posicion en funcion del angulo
   p(1) = p(1) + v*cos(u); %eje x
   p(2) = p(2) + v*sin(u); %eje y
   
   plot(p(1),p(2),'bo')
   
   i=i+1;
    
end

psi_fin = psi;
p_fin = p;

end %function


