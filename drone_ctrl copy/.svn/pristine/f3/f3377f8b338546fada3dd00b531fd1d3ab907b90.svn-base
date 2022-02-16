% complementary filter -
% pressure height and accelerometer
close all
clear
%%
tau = 0.5;
beta = 6;
GL = tf([1],[tau 1]);
G2 = tf([tau],[1 1/beta]);
% total system should be 1
Gtot = (tf([1],[1 0 0]) + G2)*GL;
%% bodeplot
figure(100)
Gd = tf([1 0 0],[0.0001 0.01*2 1])
bode(Gtot*Gd)
grid on
%% step respons
figure(102)
step(Gtot*Gd, 20)
grid on
%% C2D
T = 0.0025;
accPreFilt = c2d(G2,T,'tustin')
lowPass = c2d(GL,T,'tustin')
%%
roll = 10*pi/180;
pitch = 0*pi/180;
yaw = 0*pi/180;
roll = 0.3;
pitch = 0.2;
yaw = 0.1;
Rr = [1, 0, 0; ...
      0, cos(roll), -sin(roll); ...
      0, sin(roll), cos(roll)];
Rp = [cos(pitch), 0, sin(pitch); ...
      0,          1,     0; ...
      -sin(pitch), 0, cos(pitch)];
Ry = [cos(yaw), -sin(yaw), 0; ...
      sin(yaw), cos(yaw), 0; ...
      0,              0,       1];
R = Ry*Rp*Rr
Ri = inv(R)

a = Ri * [0;0;1]

aw = R*a
