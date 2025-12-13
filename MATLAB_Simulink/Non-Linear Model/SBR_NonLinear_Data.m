%% Self-balancing robot parameters v2

% Using stand-alone model of DC motor and inverse pendulum to derive
% transfer function

clc;
clearvars;

% Motor: DFRobot DC Transmission Motor with Encoder - 6V - 160RPM

% Inverse pendulum data
g = 9.81;   % [m/s^2] gravity acceleration

% DC motor parameters
Ra = 3.5;    % [ohm] measured resistance
La = 50e-6;   % [H] to be measured
KePhi = 0.336; % [V*s] torque costant (Va_max-Ra*Ia)*1/Omega_max 

k_gear = 120;   % gear ratio
k_eff = 0.8; % efficiency of the gear ratio

% Body parameters
m_body = 200e-3; % [kg] mass of the whole system
h_body = 20e-3; % [m] distance of the center of mass from the axle
b_body = 10e-3; % [N*m*s/rad] damping coefficient (estimated)
J_body = 1/12*m_body*h_body^2; % [kg*m^2] moment of inertia of the body


% Wheel parameters
r_w = 77.1e-3; % [m] wheels radius
h_w = 10e-3; % [m] wheels radius
m_w = 20e-3; % [kg] wheels mass
mu_w = 0.2; % wheel friction coefficient

Jw = (0.5 * m_w * r_w^2) + (1/12 * m_w * h_w^2); %[kg*m^2] 

% Code/microcontroller parameters
Vdd = 3.3; %[V] Microcontroller power supply voltage
f_pwm = 100; %[Hz] PWM Frequency 
n_pwm = 8; %[bit] PWM Resolutiom
Q_pwm = Vdd/(2^n_pwm); %[s] PWM LSB
powerConv_kp = 6/Vdd; %[V/V] Voltage gain of the power converter (H bridge)

s = tf('s');
G_i = 1/(Ra-s*La);