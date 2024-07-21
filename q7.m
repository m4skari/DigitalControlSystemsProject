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

[num,den] = ss2tf(A,B,C,D);
Gdiscrete = c2d(tf(num,den),0.01,'zoh')
[Nd, Dd]=tfdata(Gdiscrete);
zpk(Gdiscrete)

syms f1 f2 f3

eqns = [ f1+f2+f3==1 , 3.085*f1-f2+f3/3.085==0 , -f1+2.54+f3/1.54==0];
vars = [f1 f2 f3];
f=solve(eqns,vars);

f1 = double(f.f1)
f2 = double(f.f2)
f3 = double(f.f3)
alpha = f3/1.04
beta1 = f1
beta2 = f3/2.293/f1
