% Test de Carrot Chase algorithm
clear all
close all
clc
% pi = 3.14159265358979323846;

% Waypoint actual W(i)
W_act = [2 2];              % W_i = (xi, yi)
% Waypoint siguiente W(i+1)
W_sig = [100 100];			% W_i+1 = (xi+1, yi+1)

WP = [  2.0    2.0;
      502.0  502.0;
      560.6  643.4;
      702.0  702.0;
      843.4  643.4;
      902.0  502.0;
      843.4  360.6;
      702.0  302.0;
      560.6  360.6];

% Posicion inicial del UAV
p_ini = [0 0];			% p = (x y);
% Heading angle inicial (YAW)
psi = pi/2;

figure(1)
hold on
grid on
% p_fin = simuladorVuelo(p_ini,psi, W_act, W_sig);
n = size(WP,1)-1;
for i=1:n
   
   W_act = [WP(i,1) WP(i,2)];
   W_sig = [WP(i+1,1) WP(i+1,2)];
   [ p_fin, psi_fin ] = simuladorVuelo(p_ini, psi, W_act, W_sig);
   p_ini = p_fin;
   psi = psi_fin;

end



