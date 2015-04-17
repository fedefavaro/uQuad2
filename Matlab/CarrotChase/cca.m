% Test de Carrot Chase algorithm
clear all
close all
clc
% pi = 3.14159265358979323846;

% Waypoint actual W(i)
% Waypoint siguiente W(i+1)

% WP = [  2.0    2.0;
%       502.0  502.0;
%       560.6  643.4;
%       702.0  702.0;
%       843.4  643.4;
%       902.0  502.0;
%       843.4  360.6;
%       702.0  302.0;
%       560.6  360.6; 
%       560.6    2.0;
%      1000      2.0;
%      1202.0  502.0;
%      1260.6  643.4;
%      1402    702.0;
%      1543    643.4;
%      1602    502.0;
%      1543    360.6;
%      1402    302.0;
%      1260    360.6; 
%      1260   -150.0;
%         0      0]; 
  

% WP = [  0    0;
%       100   100;
%       100   117.5;
%       116.5 117.5;
%       116.5 157.5;
%       100  157.5;
%       100   175;
%       210   175;
%       210   157.5;
%       193.5 157.5;
%       193.5 117.5;
%       210   117.5;
%       210   100;
%       100   100];

  
WP = importdata('trayectoria_discreta.txt');

% Posicion inicial del UAV
p_ini = [0 0];			% p = (x y);
% Heading angle inicial (YAW)
psi_ini = pi;

figure(1)
hold on
grid on

n = size(WP,1)-1;

for i=1:n
   
   % Distancia
   %d = sqrt( ( W_sig(1) - W_act(1) )^2 + ( W_sig(2) - W_act(2) )^2 );
   d = sqrt( ( WP(i+1,1) - WP(i,1) )^2 + ( WP(i+1,2) - WP(i,2) )^2 );

   %plot(W_act(1), W_act(2), 'ro')
   plot(WP(i,1), WP(i,2), 'ro')
   %plot(W_sig(1), W_sig(2), 'ro')
   plot(WP(i+1,1), WP(i+1,2), 'ro')
   
   line = 0:0.01:d;
   %theta = atan2( W_sig(2) - W_act(2), W_sig(1) - W_act(1));
   theta = atan2( WP(i+1,2) - WP(i,2), WP(i+1,1) - WP(i,1));
   
   %x = W_act(1) + line*cos(theta);
   x = WP(i,1) + line*cos(theta);
   %y = W_act(2) + line*sin(theta);
   y = WP(i,2) + line*sin(theta);
   
   plot(x,y,'r-')

end

pause

for i=1:n
   
   W_act = [WP(i,1) WP(i,2)];
   W_sig = [WP(i+1,1) WP(i+1,2)];
   [ p_fin, psi_fin ] = simuladorVuelo(p_ini, psi_ini, W_act, W_sig);
   p_ini = p_fin;
   psi_ini = psi_fin;

end

hold off


