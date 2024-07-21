clear all
close all
syms x1 x2 x3 u;
%%%%%%%%%%%%%%%%
R = 10; 
L = 0.115; 
g = 9.8; 
M = 0.0844;
c = 7000;  
ystar = 0.002;

eq1= x2 == 0;
eq2= -g+c*x3^2/(0.0072-x1)/M == 0;
eq3= 1/L*(-R*x3+u) == 0;
eq4= x1 == 0.002;
sol=solve([eq1,eq2,eq3,eq4],[x1 x2 x3 u]);

x1star = double(sol.x1)
x2star = double(sol.x2)
x3star = double(sol.x3)
ustar = double(sol.u)
%%%%%%%%%%%%%%%%%%%%%%%%
x1star = 0.002;
x2star = 0;
x3star = 0.0007839;
ustar = 0.0078;
ystar = x1star;

A = [0, 1, 0;c/M*x3star^2/(0.0072-x1star)^2, 0, c/M*x3star*2/(0.0072-x1star); 0, 0, -R/L]
B = [0 0, 1/L]';
C = [1, 0, 0];
D = 0;

x = [x1-x1star, x2-x2star, x3-x3star]';
xdot = A*x + B*u
y = C*x + D*u - ystar

s=tf('s');
gp=C*((s.*eye(3)-A)^-1)*B+D

Gp = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D);

step(gp)
