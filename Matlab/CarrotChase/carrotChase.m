function [ psi_d ] = carrotChase(p, W_act, W_sig)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% delta
delta = 5;

% Paso 2
Ru = norm( W_act - p);
theta = atan2( W_sig(2) - W_act(2), W_sig(1) - W_act(1));   % atan2(y_i+1 - y_i, x_i+1 - x_i)

% Paso 3
theta_u = atan2( p(2) - W_act(2), p(1) - W_act(1));         % atan2(y - y_i, x - x_i)
beta = theta - theta_u;

% Paso 4
R = sqrt( Ru^2 - (Ru*sin(beta))^2);

% Paso 5
s = [(R + delta)*cos(theta) + W_act(1) (R+delta)*sin(theta) + W_act(2)];

% Paso 6
psi_d = atan2(s(2) - p(2), s(1) - p(1));
%psi_d_grados = psi_d*180/pi

end %function

