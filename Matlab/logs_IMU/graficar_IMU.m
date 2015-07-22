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
% 17 T_s_IMU
% 18 T_us_IMU
% 19 alt
% 20 us_obs
% 21 us_alt


%% Grafica datos obtenidos
data = importdata('log_todo_1',' ');
% dataB = importdata('log_PD_nuevo_3',' ');

t_CC3D = data(:,1)*1000 + data(:,2)/1000; %timepo CC3D en milisegundos
t_main = data(:,10)*1000 + data(:,11)/1000; %timepo main en milisegundos
t_IMU = data(:,17)*1000 + data(:,18)/1000; %timepo main en milisegundos

figure
plot(t_main,data(:,19))
title('Altura')
xlabel('t(ms)')
ylabel('h(m)')
grid on

figure
plot(t_main,data(:,20))
title('Distancia us')
xlabel('t(ms)')
ylabel('d(m)')
grid on

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

