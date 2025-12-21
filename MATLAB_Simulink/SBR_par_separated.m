%% Self-balancing robot parameters v2

% Using stand-alone model of DC motor and inverse pendulum to derive
% transfer function

clc;
clearvars;

% Motor: DFRobot DC Transmission Motor with Encoder - 6V - 160RPM

% Inverse pendulum data
m = 0.1;    % [Kg] total mass, considering also motors and battery
g = 9.81;   % [m/s^2] gravity acceleration
l = 0.08;    % [m] length of the pendulum
J = m*l^2;  % [Kg*m^2] moment of inertia of inverse pendulum

% DC motor parameters
R = 3.5;    % [ohm] measured resistance
L = 50e-6;   % [H] to be measured, stimata da ChatGPT
KePhi = 0.336; % [V*s] torque costant (Va_max-Ra*Ia)*1/Omega_max 
k_gear = 120;   % gear ratio

%% Transfer functions

s = tf('s');

% between torque to angle of the inverse pendulum
G_torq_ang = (1/J) *(1/(s^2 - g/l));
% figure('Name', 'Transfer function of the inverse pendulum');
% bode(G_ang_torq);

% between voltage to torque of the DC motor
G_volt_torq = KePhi * k_gear/(R + L*s);  % Transfer function from voltage to torque
% figure('Name', 'Transfer function of the DC motor');
% bode(G_torq_volt);

% total transfer function
G_sys = G_volt_torq*G_torq_ang;

%% Requirements: tempo di salita e overshoot

ts = 50e-3; % rise time
M = 0.15;  % overshoot: percentuale 7.5/6 (massima tensione che può accettare) = 0.2 -> margine di sicurezza

fct = 2/(ts*2*pi);
pmt = 1.04 - 0.8*M;

opt = pidtuneOptions;
opt.PhaseMargin = 180*pmt/pi;

gi= pidtune(G_sys, 'pid', 2*pi*fct, opt);    % 'pidf' così ho il filtro in HF per simulare in Simulink

G_reg = gi.Kp + gi.Ki/s + gi.Kd*s/(1+s*gi.Tf);

fprintf('PID parameters: Kp %.4f, Ki %.4f, Kd %.4f\n', gi.Kp, gi.Ki, gi.Kd);

[Nreg, Dreg] = tfdata(G_reg, 'v');    % 'v' per averceli in vettore
% margin(G_reg*G_sys);
% grid on;

%% Step response of the closed loop system
G_cl = feedback(G_reg*G_sys, 1);
figure('Name', 'Step response of the closed loop system');
stepSetting = RespConfig('Amplitude', 20, 'Bias', -20);
step(G_cl, stepSetting);
title('Step response of the closed loop system');
grid on;
hold on;
ylabel("Angle (degrees)");
xlabel("Time");
fontsize(gca, 13, 'points');