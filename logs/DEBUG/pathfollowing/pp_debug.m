close all
clear all
clc

data = importdata('log_debug_14');

figure
plot(data(:,13),data(:,14),'*r');
axis equal
grid on

t = data(:,11)*1000 + data(:,12)/1000;
figure
plot(t,data(:,9),'*r');