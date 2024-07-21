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

G = c2d(gp, Ts(2), 'matched')

% Plot the root locus
figure;
rlocus(G);
title('Root Locus of the Discretized System');

% Compute and display the gain margin, phase margin, and bandwidth
[GM, PM, Wcg, Wcp] = margin(G);

% Display the gain margin and phase margin
disp(['Gain Margin (dB): ', num2str(20*log10(GM))]);
disp(['Phase Margin (degrees): ', num2str(PM)]);

% Display the crossover frequencies
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcg)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcp)]);

% Display bandwidth
bw = bandwidth(G);
disp(['Bandwidth (Hz): ', num2str(bw)]);

% Display the Bode plot with gain and phase margins
figure;
margin(G);
title('Bode Plot with Gain and Phase Margins');

% Display the step response
figure;
step(G);
title('Step Response of the Discretized System');
