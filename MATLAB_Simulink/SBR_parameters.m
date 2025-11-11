%% Self-balancing robot parameters

% Motor: DFRobot DC Transmission Motor with Encoder - 6V - 160RPM

% Max stall torque = 0.08 [Nm] con max stall current = 2.8 [A] => k =
% 0.08/2.8 da capire se il rapporto di trasmissione è compreso

% From datasheet: No-load current @ 6V: 0.17A -> trovo da qui la R

m = 2;         % massa del robot [kg]
g = 9.81;      % costante gravitazionale [m/s^2]
l = 0.4;       % altezza robot [m]
k = 0.08/2.8;  % costante di coppia del motore [Nm/A]
R = 3.5;       % resistenza motore -> misurato con tester [ohm]
L = 1e-2;      % induttanza motore [H] -> da misurare
beta = 1e-5;   % coefficiente di attrito viscoso aria [Nms]
J = m*l^2;     % momento d'inerzia del sistema [kg*m^2]

max_v = 6*1.2;   % tensione massima applicabile

%% Controller design (PID)

% mi trovo la matrice "A", vettori "b" e "c" per trovare la funzione di trasferimento del sistema
s = tf('s');

A = [-R/L 0 -k/L; 0 0 1; k/(m*l^2) -g/l*cos(pi) -beta/(m*l^2)];

b = [1/L; 0; 0];

c = [0, 1, 0];

% identity matrix
I = [1 0 0; 0 1 0; 0 0 1];

% Calculations
MfD = s*I - A; % matrix for determinant calculations
pol_car = s*( MfD(1,1)*MfD(3,3) - MfD(1,3)*MfD(3,1) ) + MfD(1,1)*MfD(3,2);

% transfer function

G = tf(c*(s*I-A)^(-1)*b);
p = pole(G);

% negletting the more distance pole
[num_W, den_W] = tfdata(G, 'v');
G_appr = num_W(4)/((s - p(2))*(s - p(3)));

%% Requirements: tempo di salita e overshoot

ts = 1e-3; % rise time
M = 0.15;  % overshoot: percentuale 7.5/6 (massima tensione che può accettare) = 0.2 -> margine di sicurezza

fct = 2/(ts*2*pi);
pmt = 1.04 - 0.8*M;

opt = pidtuneOptions;
opt.PhaseMargin = pmt;

[gi, info] = pidtune(G_appr, 'pidf', 2*pi*fct, opt);    %'pidf' così ho il filtro nella parte derivativa

reg = gi.Kp + gi.Ki/s + gi.Kd*s/(1+s*gi.Tf);
[Nreg, Dreg] = tfdata(reg, 'v');    % 'v' per averceli in vettore
margin(reg*G_appr);
grid on;