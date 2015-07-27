close all
clear all
clc

% Log - 
% 1  T_s_act 
% 2  T_us_act
% 3  roll
% 4  pitch
% 5  yaw
% 6  C_roll 
% 7  C_pitch 
% 8  C_yaw
% 9  C_throttle 
% 10 T_s_main
% 11 T_us_main 
% 12 pos.x 
% 13 pos.y 
% 14 pos.z 
% 15 yaw_d
% 16 u_yaw
% 17 h_d
% 18 u_h
% 19 T_s_IMU
% 20 T_us_IMU
% 21 alt
% 22 us_obs
% 23 us_alt


%% Grafica datos obtenidos
data = importdata('log_canetigiro2_13',' ');

t_CC3D = data(:,1)*1000 + data(:,2)/1000; %timepo CC3D en milisegundos
t_main = data(:,10)*1000 + data(:,11)/1000; %timepo main en milisegundos
t_IMU = data(:,19)*1000 + data(:,20)/1000; %timepo main en milisegundos

display('promedio de tiempo del main:');
display(mean(diff(t_main(162:length(t_main)))));

display('promedio de tiempo de CC3D:');
display(mean(diff(t_CC3D(162:length(t_CC3D)))));

display('promedio de tiempo de IMU:');
display(mean(diff(t_IMU(162:length(t_IMU)))));


figure
hold on
plot(t_main,data(:,3)*180/3.1416,'*r')
plot(t_main,data(:,4)*180/3.1416,'*b')
plot(t_main,data(:,5)*180/3.1416,'*g')
title('Euler angles')
xlabel('time (ms)')
legend('roll(deg)','pitch(deg)','yaw(deg)')
grid on
hold off


figure
hold on
plot(t_main,data(:,9),'*r')
title('Throttle')
xlabel('time (ms)')
grid on
hold off

figure
plot(t_main,data(:,17))
title('Altura deseada')
xlabel('t(ms)')
ylabel('h(m)')
grid on

figure
plot(t_main,data(:,18))
title('senal de control altura')
xlabel('t(ms)')
ylabel('h(m)')
grid on


figure
hist(data(:,21),50)
title('Histograma Altura baro')

display('promedio de muestras baro:');
display(mean(data(:,21)));

display('desviaicion estandar muestras baro:');
display(var(data(:,21)));

figure
hold on
plot(t_main,data(:,21),'b')
title('Altura baro')
xlabel('t(ms)')
ylabel('h(m)')
grid on

alpha=0.1;  %coef del filtro
datos_filtrados = zeros(length(data(:,21)),1);
datos_filtrados(1) = data(1,21);
for i=2:length(data(:,21))
    datos_filtrados(i) = data(i,21)*alpha+(1-alpha)*datos_filtrados(i-1);    
end
plot(t_main,datos_filtrados,'r')
legend('datos reales','datos filtrados')
hold off

% figure
% hist(datos_filtrados,50,'r')
% title('Histograma Altura baro filtrado')

figure
plot(t_main,data(:,22))
title('Distancia us frente')
xlabel('t(ms)')
ylabel('d(m)')
grid on

figure
hold on
plot(t_main,data(:,23))
title('Distancia us altura')
xlabel('t(ms)')
ylabel('d(m)')
grid on

% alpha=0.25;  %coef del filtro
% umbral = 0.08; %umbral para descartar muestras
% datos_filtrados = zeros(length(data(:,23)),1);
% datos_filtrados(1) = data(1,23);
% for i=2:length(data(:,23))
%     if ( data(i,23) > (1+umbral)*data(i-1,23) ) ||  ( data(i,23) < (1-umbral)*data(i-1,23) )
%         data(i,23) = data(i-1,23);
%         datos_filtrados(i) = datos_filtrados(i-1); 
%     else
%         datos_filtrados(i) = data(i,23)*alpha+(1-alpha)*datos_filtrados(i-1);    
%     end
% end
% plot(t_main,datos_filtrados,'r')
% legend('datos reales','datos filtrados')
% hold off

figure
plot(t_CC3D)
title('tiempo de muestreo CC3D')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on


figure
plot(t_CC3D)
title('tiempo de muestreo CC3D')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on


figure
plot(diff(t_CC3D))
title('Derivada del tiempo de muestreo CC3D')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(t_main)
title('tiempo de muestreo main')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on


figure
plot(diff(t_main))
title('Derivada del tiempo de muestreo main')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(t_IMU)
title('tiempo de muestreo IMU')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(diff(t_IMU))
title('Derivada del tiempo de muestreo IMU')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

