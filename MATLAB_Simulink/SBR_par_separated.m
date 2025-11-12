%% Self-balancing robot parameters v2

% Using stand-alone model of DC motor and inverse pendulum to derive
% transfer function

% Motor: DFRobot DC Transmission Motor with Encoder - 6V - 160RPM

% Inverse pendulum data
m = 0.3;    % [Kg] total mass, considering also motors and battery
g = 9.81;   % [m/s^2] gravity acceleration
l = 0.5;    % [m] length of the pendulum
J = m*l^2;  % [Kg*m^2] moment of inertia of inverse pendulum

% DC motor parameters
R = 3.5;    % [ohm] measured resistance
L = 1e-2;   % [H] to be measured
KePhi = 0.336; % [V*s] torque costant (Va_max-Ra*Ia)*1/Omega_max 
k_gear = 120;   % gear ratio

%% Transfer functions

s = tf('s');

% between torque to angle of the inverse pendulum
G_ang_torq = 1/(J*s^2 - J*g/l);
figure('Name', 'Transfer function of the inverse pendulum');
bode(G_ang_torq);

% between voltage to torque of the DC motor
G_torq_volt = KePhi/(R + L*s);  % Transfer function from voltage to torque
figure('Name', 'Transfer function of the DC motor');
bode(G_torq_volt);