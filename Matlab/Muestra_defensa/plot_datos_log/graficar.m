close all
clear all
clc

% %% Grafica Trayectoria ideal
% figure
% hold on
% grid on
% axis equal
% title('Trayectoria 2D')
% 
% data = importdata('cuadrado.txt');
% n = data(1);
% 
% for i=0:(size(data,1)/19)-1
%     
%     % Radio de cfas
%     r = data(2+19*i);
%     % Way point inicial
%     xi = data(3+19*i);
%     yi = data(4+19*i);
%     alpha = data(5+19*i);
%     % Way point final
%     xf = data(6+19*i);
%     yf = data(7+19*i);
%     beta = data(8+19*i);
%     % Tipo de curva de Dubins (R=0, L=1)
%     if ((data(9+19*i) == 0) || (data(9+19*i) == 1) || (data(9+19*i) == 5))
%         cfa_i = 1;
%     else
%         cfa_i = 0;
%     end
%     if ((data(9+19*i) == 0) || (data(9+19*i) == 3) || (data(9+19*i) == 5))
%         cfa_f = 1;
%     else
%         cfa_f = 0;
%     end
%     % Coordenadas cfa inicial
%     xci = data(10+19*i);
%     yci = data(11+19*i);
%     % Coordenadas recta
%     xri = data(12+19*i);
%     yri = data(13+19*i);
%     xrf = data(14+19*i);
%     yrf = data(15+19*i);
%     % Coordenadas cfa final
%     xcf = data(16+19*i);
%     ycf = data(17+19*i);
%     % Angulo de arco de cfa inicial
%     l1 = mod2pi(data(18+19*i));
%     % Largo tramo recto
%     l2 = data(19+19*i);
%     % Angulo de arco de cfa final
%     l3 = mod2pi(data(20+19*i));        
% 
% % Generacion de vectores para graficar
% 
%  if cfa_i == 0
%      theta1 = linspace(alpha+(pi/2), alpha+(pi/2)-l1, 500);
%  end
%  if cfa_i == 1
%      theta1 = linspace(alpha-(pi/2), alpha-(pi/2)+l1, 500);
%  end
%  x1 = xci + r*cos(theta1);
%  y1 = yci + r*sin(theta1);
% 
% if cfa_f == 0
%     theta2 = linspace(beta+(pi/2)+l3, beta+(pi/2), 500);
% end
% if cfa_f == 1
%     theta2 = linspace(beta-(pi/2)-l3, beta-(pi/2), 500);
% end
% x2 = xcf + r*cos(theta2);
% y2 = ycf + r*sin(theta2);
% 
% x3 = linspace(xri,xrf,500);
% y3 = linspace(yri,yrf,500);
% 
% plot(x1,y1,'r')
% plot(x2,y2,'r')
% plot(x3,y3,'r')
% drawnow;
% 
% end


%% Grafica datos obtenidos
data = importdata('log_pre_muestra',' ');
% dataB = importdata('log_PD_nuevo_3',' ');

%posicion -xy
plot(data(:,12),data(:,13),'b')
% plot(dataB(:,12),dataB(:,13),'g')

t = data(:,1)*1000 + data(:,2)/1000; %timepo en milisegundos
t2 = data(:,10)*1000 + data(:,11)/1000; %timepo en milisegundos
% tB = dataB(:,10)*1000 + dataB(:,11)/1000; %timepo en milisegundos
% t3 = data2(:,10)*1000 + data2(:,11)/1000; %timepo en milisegundos
% t4 = data3(:,10)*1000 + data3(:,11)/1000; %timepo en milisegundos

figure
hold on
plot(t2,data(:,3)*180/3.1416,'*r')
plot(t2,data(:,4)*180/3.1416,'*b')
plot(t2,data(:,5)*180/3.1416,'*g')
title('Euler angles')
xlabel('time (ms)')
legend('roll(deg)','pitch(deg)','yaw(deg)')
grid on
hold off

figure
hold on
plot(t2,data(:,5)*180/3.1416,'b')
plot(t2,data(:,15)*180/3.1416,'r')
plot(t2,data(:,16),'k')
% plot(u*25/11 + 1500);
title('Yaw medido vs yaw deseado')
xlabel('time (ms)')
ylabel('Yaw (deg)')
legend('yaw medido','yaw deseado','yaw punto')
grid on
hold off



figure
hold on
plot(t2,data(:,16),'r')
yaw_punto_real = diff(data(:,5)*180/3.1416)/0.050;
yaw_punto_real = [yaw_punto_real(1); yaw_punto_real];
plot(t2,yaw_punto_real,'b')
title('yaw punto deseado vs yaw punto real 3.5')
legend('yaw punto deseado','yaw punto real')
xlabel('time (ms)')
ylabel('Yaw (deg/s)')
grid on
hold off
% 
% figure
% hold on
% plot(tB,dataB(:,16),'r')
% yaw_punto_real2 = diff(dataB(:,5)*180/3.1416)/0.050;
% yaw_punto_real2 = [yaw_punto_real2(1); yaw_punto_real2];
% plot(tB,yaw_punto_real2,'b')
% title('yaw punto deseado vs yaw punto real 2.7')
% legend('yaw punto deseado','yaw punto real')
% xlabel('time (ms)')
% ylabel('Yaw (deg/s)')
% grid on
% hold off
% 
% figure
% hold on
% plot(t4,data3(:,16),'r')
% yaw_punto_real3 = diff(data3(:,5)*180/3.1416)/0.050;
% yaw_punto_real3 = [yaw_punto_real3(1); yaw_punto_real3];
% plot(t4,yaw_punto_real3,'b')
% title('yaw punto deseado vs yaw punto real 17 %')
% legend('yaw punto deseado','yaw punto real')
% xlabel('time (ms)')
% ylabel('Yaw (deg/s)')
% grid on
% hold off

figure
plot(t2,'*b')
title('Tiempo de muestreo')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(diff(t2))
title('Derivada del tiempo de muestreo')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(diff(t))
title('Derivada del tiempo de medida')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
hold on
plot(t2,data(:,6),'*r')
plot(t2,data(:,7),'*b')
plot(t2,data(:,8),'*g')
plot(t2,data(:,9),'*k')
title('Comandos')
xlabel('time (ms)')
legend('Roll','Pitch','Yaw','Throttle')
grid on
hold off

% figure
% plot(diff(t2))
% title('Derivada del tiempo de loop')
% xlabel('numero de muestra')
% ylabel('tiempo(ms)')
% grid on

% figure
% plot(t-t2)
% title('Diferencia entre tiempo de loop y tiempo de muestreo')
% xlabel('numero de muestra')
% ylabel('tiempo(ms)')
% grid on
% 
% figure
% plot(diff(t)-diff(t2))
% title('Diferencia entre derivadas de tiempo de loop y tiempo de muestreo')
% xlabel('numero de muestra')
% ylabel('tiempo(ms)')
% grid on

