close all
clear all
clc

data = importdata('log_debug_1.txt',' ');

t = data(:,1).*1000 + data(:,2)./1000;


figure(1)
hold on

plot(t,data(:,5),'*k')


hold off

