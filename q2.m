clear all
close all

x1star = 0.002;
x2star = 0;
x3star = 0.0007839;
ustar = 0.0078;
ystar = x1star;

c = 7000; 
M = 0.0844;     
R = 10;    
L = 0.115;      


A = [0, 1, 0; 
     c/M*x3star^2/(0.0072-x1star)^2, 0, c/M*x3star*2/(0.0072-x1star); 
     0, 0, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0];
D = 0;


s = tf('s');
gp = C*((s.*eye(3)-A)^-1)*B + D;


figure;
step(gp);
title('Open-Loop Step Response');

opts = pidtuneOptions('DesignFocus', 'disturbance-rejection','PhaseMargin', 18); 

[Gc, info] = pidtune(gp, 'pid', opts); 

% Display PID Controller Gains
disp('PID Controller Gains:')
disp(Gc)

% Closed-Loop System
T = feedback(Gc*gp, 1); % Unity feedback assumed

% Closed-Loop Analysis
figure;
step(T);
stepinfo(T)
title('Closed-Loop Step Response with PID Controller');
