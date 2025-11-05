%% Self-balancing robot parameters

% mancano le unità di misura e capire K

m = 60;        % massa del robot
g = 9.81;      % costante gravitazionale
l = 0.7;       % altezza robot
k = 50;        % costante di coppia del motore
R = 8;         % resistenza motore
L = 1e-2;      % induttanza motore
beta = 1e-5;   % coefficiente di attrito viscoso aria 
J = m*l^2;     % momento d'inerzia del sistema

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

W = tf(c*(s*I-A)^(-1)*b);
p = pole(W);

% negletting the more distance pole
[num_W, den_W] = tfdata(W, 'v');
W_appr = num_W/((s - p(2))*(s - p(3)));

bodeplot(W_appr);