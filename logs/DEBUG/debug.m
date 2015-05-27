close all
clear all
clc

data = importdata('log_debug_7.txt',' ');

t = data(:,1)*1000 + data(:,2)/1000; %timepo en milisegundos

t2 = data(:,6)*1000 + data(:,7)/1000; %timepo en milisegundos

figure
plot(diff(t))
grid on


figure
plot(diff(t2))
grid on