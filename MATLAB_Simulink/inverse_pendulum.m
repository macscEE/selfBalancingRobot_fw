%% Inverse pendulum model
% This script defines the dynamics of an inverse pendulum

m = 1;       % mass of the pendulum [kg]
l = 0.5;     % length of the pendulum [m]
g = 9.81;    % gravitational acceleration [m/s^2]

% transfer function of the inverse pendulum
s = tf('s');
G_p = (1/m*l^2)/(s^2 - g/l);

% Display the transfer function
disp(G_p);

% Bode plot
figure('Name','Transfer function of the inverse pendulum:');
bode(G_p);
grid on;    