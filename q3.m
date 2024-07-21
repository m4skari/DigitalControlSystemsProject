clear all
close all

% System Parameters
x1star = 0.002;
x2star = 0;
x3star = 0.0007839;
ustar = 0.0078;
ystar = x1star;
c = 7000; 
M = 0.0844;     
R = 10;    
L = 0.115; 

% System Matrices
A = [0, 1, 0; 
     c/M*x3star^2/(0.0072-x1star)^2, 0, c/M*x3star*2/(0.0072-x1star); 
     0, 0, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0];
D = 0;

% Continuous-time Transfer Function
s = tf('s');
gp = C*((s.*eye(3)-A)^-1)*B + D

% Continuous-time PID Controller Design
opts = pidtuneOptions('DesignFocus', 'disturbance-rejection','PhaseMargin', 18); 
[Gc, info] = pidtune(gp, 'pid', opts)

% Display PID Controller Gains
disp('Continuous-time PID Controller Gains:')
disp(Gc)

% Sampling Rates
Ts1 = 0.001;   % Fast sampling rate
Ts2 = 0.01;   % Moderate sampling rate
Ts3 = 0.1;    % Slow sampling rate

% --- Zero-Pole Matching Discretization ---
Gzpm1 = c2d(Gc, Ts1, 'matched')
Gzpm2 = c2d(Gc, Ts2, 'matched');
Gzpm3 = c2d(Gc, Ts3, 'matched');

% --- Bilinear Transformation Discretization ---
Gzbt1 = c2d(Gc, Ts1, 'tustin')
Gzbt2 = c2d(Gc, Ts2, 'tustin');
Gzbt3 = c2d(Gc, Ts3, 'tustin');

% --- Closed-loop Systems and Analysis --- 
% Analyze for each sampling rate and method
analyzeSystem(gp,Gzpm1, Ts1, 'Zero-Pole Matching');
analyzeSystem(gp,Gzpm2, Ts2, 'Zero-Pole Matching');
analyzeSystem(gp,Gzpm3, Ts3, 'Zero-Pole Matching');

analyzeSystem(gp,Gzbt1, Ts1, 'Bilinear Transform');
analyzeSystem(gp,Gzbt2, Ts2, 'Bilinear Transform');
analyzeSystem(gp,Gzbt3, Ts3, 'Bilinear Transform');

% --- Comparison with Continuous-Time System ---
% Select the best discrete system based on plots and stepinfo
% (Assuming Gzpm1 and Ts1 are chosen - replace if needed)
t_cont = 0:0.001:1;  % Time vector for continuous system
figure;
step(feedback(Gc*gp, 1), t_cont); 
hold on;
step(feedback(Gzpm1*c2d(gp, Ts1), 1), t_cont);
legend('Continuous', 'Discrete (ZPM, Ts = 0.001s)');
title('Comparison with Continuous System');

% --- Time-Domain Characteristic Analysis ---
info_cont = stepinfo(feedback(Gc*gp, 1)); 
info_disc = stepinfo(feedback(Gzpm1*c2d(gp, Ts1), 1)); 

disp('--- Continuous System ---')
disp(info_cont)
disp('--- Discrete System (Best Choice) ---')
disp(info_disc) 

% --- Function Definition MUST go at the end ---
function analyzeSystem(gp,Gd, Ts, method)
    sys_cl = feedback(Gd*c2d(gp, Ts), 1);
    t_disc = 0:Ts:1;  % Adjust time span if needed
    figure;
    step(sys_cl, t_disc);
    hold on;
    title(['Closed-Loop Step Response - ', method, ', Ts = ', num2str(Ts), 's']);
    info_disc = stepinfo(sys_cl);
    disp(['--- ', method, ', Ts = ', num2str(Ts), 's ---'])
    disp(info_disc)
end
