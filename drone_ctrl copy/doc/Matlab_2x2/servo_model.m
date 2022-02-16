%% RC serco
close all
clear
%% motor
servo_Km = 0.0064;
servo_Ra = 0.5;
servo_L = 0.001;
servo_Vcc = 6;
% limits
servo_stall = 0.44; % Nm
servo_speed = 0.09; % sec for 60 deg
servo_Gear = 300;
servo_frik = 0.00001; % friction Nm / (rad/s)
%% dummy values
servo_vkp = 1;
servo_vtd = 0;
servo_vti = 1;
servo_val = 0;
%
servo_kp = 0;
servo_td = 0;
servo_ti = 1;
servo_al = 0;

%% controller design
model = 'servo_sim';
load_system(model);
open_system(model);
ios(1) = linio('servo_sim/RC_servo/servo_vel',1,'openinput');
ios(2) = linio('servo_sim/RC_servo/servo_rate',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [2];
% Linearize the model
sys = linearize(model,ios,op);
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
figure(110)
hold off
w = logspace(-2,6,1000);
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gsv = tf(num, den)
  bode(Gsv,w);
  hold on
end
grid on
%% roll controller
servo_val = 0.1;
servo_vgm = 80;
servo_vNi = 1;
[sw, servo_vkp, servo_vti, servo_vtd] = findpid(Gsv, servo_vgm, servo_vNi, servo_val)

%
