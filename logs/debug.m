close all
clear all
clc

data = importdata('log_controlPD_4.txt',' ');

t = data(:,1)*1000 + data(:,2)/1000; %timepo en milisegundos

t2 = data(:,11)*1000 + data(:,12)/1000; %timepo en milisegundos

figure
hold on
plot(t,data(:,3),'*r')
plot(t,data(:,4),'*b')
plot(t,data(:,5),'*g')
title('Euler angles')
xlabel('time (ms)')
legend('roll(deg)','pitch(deg)','yaw(deg)')
grid on
hold off

figure
plot(t,data(:,6),'*r')
title('Yaw punto')
xlabel('time (ms)')
grid on

figure
plot(diff(t))
title('Derivada del tiempo de muestreo')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on


figure
hold on
plot(t,data(:,7),'*r')
plot(t,data(:,8),'*b')
plot(t,data(:,9),'*g')
plot(t,data(:,10),'*k')
title('Comandos')
xlabel('time (ms)')
legend('Roll','Pitch','Yaw','Throttle')
grid on
hold off

figure
plot(diff(t2))
title('Derivada del tiempo de loop')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(t-t2)
title('Diferencia entre tiempo de loop y tiempo de muestreo')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

figure
plot(diff(t)-diff(t2))
title('Diferencia entre derivadas de tiempo de loop y tiempo de muestreo')
xlabel('numero de muestra')
ylabel('tiempo(ms)')
grid on

