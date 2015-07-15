close all
clear all
clc

% Log - 
% 1  T_s_act 
% 2  T_us_act
% 3  roll
% 4  pitch
% 5  yaw


%% Grafica datos obtenidos
data = importdata('log_uavtalk_parser',' ');
% dataB = importdata('log_PD_nuevo_3',' ');

t_CC3D = data(:,1)*1000 + data(:,2)/1000; %timepo CC3D en milisegundos

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


