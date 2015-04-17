function [ p_fin, psi_fin ] = simuladorVuelo(p, psi, W_act, W_sig)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%Ganancia contorl
k = 1.06;
% Velocidad
v = 0.01;                     % m/s
% Grafico posicion inicial
plot(p(1), p(2), 'kx')

% Distancia entre waypoints
%d = sqrt( ( W_sig(1) - W_act(1) )^2 + ( W_sig(2) - W_act(2) )^2 );

% contador para saber si estoy yendo para cualqueir lado
cont = 0;

Tau = 0.1*norm(W_sig - W_act);
dist_left = norm(W_sig - p);
dist_left_Prev = 0;
while dist_left > Tau
   
   %Simulo ruido
   %psi = awgn(psi, 90, 'measured');
   %p = awgn(p, 60, 'measured');
   
   % Pasos 1 - 6
   psi_d = carrotChase(p, W_act, W_sig);
   % Paso 7 - control
   u = k*(psi_d - psi);
   %%%%%%%%%%%%%%%%%%%%%%
   % Modelo simplificado
   %%%%%%%%%%%%%%%%%%%%%%
   % dx = v*cos(psi)    -->  xf = xo + v*cos(psi)
   % dy = v*sin(psi)    -->  yf = yf + v*sin(psi)  
   % dpsi = u           -->  psi = psi + u   
   % obtengo angulo 
   psi = psi + u;
   % Actualizo posicion en funcion del angulo
   p(1) = p(1) + v*cos(psi); %eje x
   p(2) = p(2) + v*sin(psi); %eje y
   
   plot(p(1),p(2),'bo')

   dist_left = norm(W_sig - p);
   
   if (dist_left > dist_left_Prev)
       cont = cont + 1
   end
   
   if (cont > 10)
       break
   end
   
   dist_left_Prev = dist_left;
   
end

psi_fin = psi
p_fin = p

end %function


