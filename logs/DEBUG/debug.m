close all
clear all
clc

data = importdata('log_debug_6.txt',' ');

t = data(:,1)*1000 + data(:,2)/1000; %timepo en milisegundos

figure
plot(diff(t))
grid on


