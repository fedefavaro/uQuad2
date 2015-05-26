close all
clear all
clc

data = importdata('log_debug_1.txt',' ');

t = data(:,1)*1000 + data(:,2)/1000; %timepo en milisegundos


figure
plot(t,data(:,3),'*k')
grid on
figure
plot(t,data(:,4),'*k')
grid on
figure
plot(t,data(:,5),'*k')
grid on
figure
plot(t,data(:,6),'*k')
grid on
figure
plot(diff(t))
grid on


