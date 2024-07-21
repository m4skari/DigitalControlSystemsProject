clear all;
close all;

R = 10; 
L = 0.115; 
g = 9.8; 
M = 0.0844;
c = 7000;  
ystar = 0.002;
x1star = 0.002;
x2star = 0;
x3star = 0.0007839;
ustar = 0.0078;
ystar = x1star;
% --- State-Space Representation ---
A = [0, 1, 0; c/M*x3star^2/(0.0072-x1star)^2, 0, c/M*x3star*2/(0.0072-x1star);  0, 0, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0];
D = 0;

% Continuous-time Transfer Function (you already have this)
s = tf('s');
gp = C*((s.*eye(3)-A)^-1)*B+D;

Ts = [ 0.1 0.01 0.001];

% Discretize the system
G = c2d(gp, Ts(2), 'matched');

% PID Tuning
opts = pidtuneOptions('DesignFocus', 'disturbance-rejection','PhaseMargin', 18); 
[Gc, info] = pidtune(G, 'pid', opts) 

% Display PID Controller Gains
disp('PID Controller Gains:')
disp(Gc)

% Closed-Loop System
T = feedback(Gc*G, 1); % Unity feedback assumed

% Closed-Loop Analysis
figure;
step(T);
stepinfo(T)
title('Closed-Loop Step Response with PID Controller');

% Plot the Bode Plot
figure;
bode(T);
grid on;
title('Bode Plot of the Closed-Loop System with PID Controller');

% Plot the Root Locus
figure;
rlocus(T);
title('Root Locus of the Closed-Loop System with PID Controller');

% Display Gain and Phase Margins
[GM, PM, Wcg, Wcp] = margin(T);
disp(['Gain Margin (dB): ', num2str(20*log10(GM))]);
disp(['Phase Margin (degrees): ', num2str(PM)]);
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcg)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcp)]);

% Display Bandwidth
bw = bandwidth(T);
disp(['Bandwidth (Hz): ', num2str(bw)]);

