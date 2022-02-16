%% parameters for control experiments
%% relates to hexacopter_sim.slx model
close all
clear
%% should debug plots be generated
debugPlot = 1; % set to 0 for no plot
% should result be saved in text-file
resultFile = 1; % set to 0 if not desired
filename = 'hexacopter.txt';
% simulink model
%% simulink model name
model = 'hexacopter_sim';
% NB model is used to estimate transfer function,
%    so NO step or other actuation should be activated
%    in this model (within the first 10 seconds)
%%
% The 3508-700KV MultiStar
% Specs:KV(RPM/V): 700
% Lipo cells: 3-4sMax 
% current: 360wMax Amps: 28A
% No Load Current: 0.5A/10v
% Internal Resistance: .083ohm
% Number of Poles: 14P12S (14poles 12 stators)
% Dimensions(Dia.xL):42.2 x 25.7mm
% Motor Shaft: 4mm
% prop shaft: 6mm bolt on hub or 12mm hole to hole for bolt threw style props
% Weight: 102g
% bolt hole spacing: 19mm * 25mm
%
%
motor = '3508-700KV';
Kv = 700; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
Ra = 0.083; % ohm
motorOnlyMass = 0.102; % motor mass
motorMass = motorOnlyMass + 0.085 % including esc and arm
motorInertia = (0.0422/2)^2*motorMass*0.6; % ~60% is rotating
propellerRadius = 13.5*2.54/2/100; % meters
propellerMassRaw = 0.017;
deadCurrent = 0.3;
% mass scaled up with motor inertia
propellerMass = propellerMassRaw + 3*motorInertia/propellerRadius^2; 
%% battery
cells = 4;
batVolt = 3.7*cells; % assumed battery voltage 3.7V per cell
%% drone konstants
motorCount = 6;
bodyMass = 1.2; % drone weight (with no motors) [kg]
droneMass = bodyMass + motorCount*(motorMass + propellerMassRaw); % kg
batteryMass = 0.56*2;
batteryDistanceFromCOG = -0.1; % positive up
% arm length (from center to motor, assumed circular)
arm_length = 0.35;
%% moment of inertia
IRoll = 4*(motorMass*(arm_length*sin(30*pi/180))^2) + ...
        2*(motorMass*arm_length^2) + ...
        batteryMass * batteryDistanceFromCOG^2;
IPitch = 4*(motorMass*(arm_length*sin(60*pi/180))^2) + ...
        batteryMass * batteryDistanceFromCOG^2;
IYaw   = 6*motorMass*arm_length^2;
%% limitations
roll_limit = 10; % roll and pitch angle limit (deg)
yaw_limit = 45; % yaw angle rate limit (deg/s)
minimumMotorVolt = 1.0; % lower limit to motor voltage (not to stop)
%heightVelocityLimit = 2; % m/s
%% save results to file
if resultFile
    fileID=fopen(filename,'w');
    fprintf(fileID, '# Result of drone configuration\n');
    fprintf(fileID, '# from mobotware/drone_ctrl/trunk/doc/Matlab/hexacopter.m\n');
    fprintf(fileID, '# simulink model name %s.slx\n', model);
    fprintf(fileID, '# date %s\n',date);
    fprintf(fileID, '#\n');
    fprintf(fileID, '[base]\n');
    fprintf(fileID, '# NB! all motor actuator output are\n');
    fprintf(fileID, '#     relative to this battery voltage, and\n');
    fprintf(fileID, '#     should be scaled relative to this\n');
    fprintf(fileID, 'batteryVoltage %g\n', batVolt);
    fprintf(fileID, '# Mass in kg\n');
    fprintf(fileID, 'batteryMass %g\n', batteryMass);
    fprintf(fileID, 'batteryToCOG %g\n', batteryDistanceFromCOG);
    fprintf(fileID, '# Drone mass exclusive of battery (kg)\n');
    fprintf(fileID, 'droneMass %g\n', droneMass);
    fprintf(fileID, '# Moment of inertia [Nm^2]\n');
    fprintf(fileID, 'iRoll %g\n', IRoll);
    fprintf(fileID, 'iPitch %g\n', IPitch);
    fprintf(fileID, 'iYaw %g\n', IYaw);
    fprintf(fileID, '#\n');
    fprintf(fileID, '[motor]\n');
    fprintf(fileID, 'motor %s\n', motor);
    fprintf(fileID, 'resistance %g\n', Ra);
    fprintf(fileID, 'kv %g\n', Kv);
    fprintf(fileID, '# mass in kg\n');
    fprintf(fileID, 'mass %g\n', motorMass);
    fprintf(fileID, 'motorCount %g\n', motorCount);
    fprintf(fileID, '# X-configureation distribution circular\n');
    fprintf(fileID, '# first motor - front right, counting clockwise\n');
    fprintf(fileID, '# arm length from center to motor axle\n');
    fprintf(fileID, 'armLength %g\n', arm_length);
    fprintf(fileID, '#\n');
    fprintf(fileID, '[propeller]\n');
    fprintf(fileID, '# radius in meter (base for calculation)\n');
    fprintf(fileID, 'radius %g\n', propellerRadius);
    fprintf(fileID, '# mass in kg (base for calculation)\n');
    fprintf(fileID, 'mass %g\n', propellerMassRaw);
    fprintf(fileID, '#\n');
    fprintf(fileID, '# limits\n');
    fprintf(fileID, '[limits]\n');
    fprintf(fileID, '# max roll and tilt angle limit (deg)\n');
    fprintf(fileID, 'rollLimit %g\n', roll_limit);
    fprintf(fileID, '# minimum motor voltage (at minimum RPM)\n');
    fprintf(fileID, 'minMotorVolt %g\n', minimumMotorVolt);
    fprintf(fileID, '#\n');
    fclose(fileID);
end
%
%% drag konst proportional to w^3
% based on 13" propeller measurements
KDrag = 1.5e-9;
%% trust constant proportional to w^2
% based on 13" propeller measurements
Ktrust = 2.7e-5;
%% hover 
heightRef = 1.0;
% sample time (not used)
Ts = 0.0025; % måling interval (sampletime) - sek
%% hover calculation
g = 9.82;
totalMass = droneMass + batteryMass;
trustHoover = totalMass * g; % Newton
trustPerPropeller = trustHoover / motorCount; % [N]
% hover calculation
hoverVel = sqrt(trustPerPropeller / Ktrust); % in radians/sec
hoverRPM = hoverVel/(2*pi) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel^3; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;
% hover control value (scale 0..1024 = 1-2ms)
hoverControl = hoverVoltage/batVolt*1024
%% height complement filter
h_tau = 1.5;  % filter pole for complement transition
h_beta = 5;   % pole replacing integrator - beta from filter pole
h_cfac = 1; % filter pole adjust for post filter only
%%
% append calculated base data
if resultFile
    fileID=fopen(filename,'a');
    fprintf(fileID, '#\n');
    fprintf(fileID, '# Drone model data\n');
    fprintf(fileID, '[model]\n');
    fprintf(fileID, '# drag proportional to rotation (rad/s) power 4 (unit Nm s^4)\n');
    fprintf(fileID, 'dragKonstant %g\n', KDrag); 
    fprintf(fileID, '# trust proportional to rotation (rad/s) squared (unit N s^2)\n');
    fprintf(fileID, 'trustKonstant %g\n', Ktrust); 
    fprintf(fileID, '# hover in RPM\n');
    fprintf(fileID, 'hoverRPM %g\n', hoverRPM); 
    fprintf(fileID, '# hover current (A)\n');
    fprintf(fileID, 'hoverCurrent %g\n', hoverCurrent); 
    fprintf(fileID, '# hover voltage (V)\n');
    fprintf(fileID, 'hoverVoltage %g\n', hoverVoltage); 
    fprintf(fileID, '#\n');
    fclose(fileID);
end
%% controller values to allow simulation
% hover
h1td = 1; % tau_d
h1al = 0.1; % alpha
h1kp = 60;   % K_P
h1ti = 1; % tau_i
% roll
rkp = 0;
rtd = 1;
rti = 1;
ral = 1;
% pitch
pkp = 0;
ptd = 1;
pal = 1;
pti = 1;
% yaw
ykp = 0;
ytd = 1;
yti = 1;
yal = 1;
% roll angle
rakp = 0;
ratd = 1;
raal = 1;
rati = 100;
% pitch angle
pakp=0;
patd = 1;
paal = 1;
pati = 100;
% yaw angle
yakp = 0;
yatd = 1;
yaal = 1;
yati = 100;
% velocity + position x
xvkp = 0;
xvtd = 1;
xval = 1;
xvti = 100;
xkp = 0;
xtd = 1;
xal = 1;
xti = 100;
% velocity + position y
yvkp = 0;
yvtd = 1;
yval = 1;
yvti = 100;
yiLim = 300;
ykp = 0;
ytd = 1;
yal = 1;
yti = 100;
% disable push
pushEnabled = 0;
heightInRef.time=[0,11]';
% fixed height of 1m for calibration
heightInRef.signals.values=[1,1]';
%% linear analysis - mixer in to trust
load_system(model);
open_system(model);
ios(1) = linio('hexacopter_sim/base_controlled_drone/trust_in',1,'openinput');
ios(2) = linio('hexacopter_sim/base_controlled_drone/drone_hardware',2,'openoutput');
setlinio(model,ios);
% Use the snapshot times 2 and 10 seconds
op = [2,4];
% Linearize the model
sys = linearize(model,ios,op);
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gtr = minreal(tf(num, den), 0.005)
end
% result:
% 2s  = tf([0.84],[1 12.7])
% 10s = tf([1.10],[1 13.5])
%%
%% linear analysis - motor to pitch angle
load_system(model);
open_system(model);
iosp(1) = linio('hexacopter_sim/base_controlled_drone/pitch_in',1,'openinput');
iosp(2) = linio('hexacopter_sim/base_controlled_drone/pitchA',1,'openoutput');
setlinio(model,iosp);
% Use the snapshot times in seconds
opp = [3, 3.5, 4, 4.5, 5.5];
% Linearize the model
sysp = linearize(model,iosp,opp);
%% get pitch transfer function
for M = 1:size(opp,2)
  [nump,denp] = ss2tf(sysp.A(:,:,M), sysp.B(:,:,M), sysp.C(:,:,M), sysp.D(:,:,M));
  Gpipo(M) = minreal(tf(nump, denp), 0.001)
end
%% bode - pitch motor to pitch angle
figure(1000)
hold off
bode(Gpipo(1))
hold on
bode(Gpipo(2))
bode(Gpipo(3))
bode(Gpipo(4))
bode(Gpipo(5))
grid on
legend('3s','3.5s', '4s','4.5s','5.5s')
title('Estimated TF from motor o/oo to pitch angle')
%% height without height velocity controller
% height control loop
hD = 0.2; % velocity drag (groundless estimate 30m/s => 6N drag)
trust2height1 = tf(1,[totalMass hD 0]);
Gh1a = Gtr * trust2height1
Gh1 = minreal(Gpipo(2))
% bode sammenligning
figure(1002)
hold off
bode(Gh1a)
hold on
bode(Gh1)
grid on
legend('physics','simulink')
% result
% Gh1a = tf([1.10],[1.13  15.41  2.70  0])
% poles:  s = 0, -13, -0.18
%% design height velocity controller
h1gm = 32;
h1al = 0.07;
h1Ni = 4;
%[hvw, hvkp, hvti, hvtd] = findpid(Ghv, hvgm, hvNi, hval)
[h1w, h1kp, h1ti, h1td] = findpid(Gh1, h1gm,  h1Ni, h1al)
% result:
% h1 Kp = 26.7, ti=1.2, td = 1.26
%% debug and save result
showResult(debugPlot,resultFile,filename, 'height_in_1_ctrl', Gh1, ...
           h1w, h1kp, h1ti, h1Ni, h1td, h1al, ...
           h1gm, 'Height velocity controll');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Model extension to roll and pitch input to roll/pitch velocity
% roll trust share
% moment motor 1
mm1 = sin(30*pi/180)/6*arm_length; %% full power and reduced arm
mm2 = arm_length/6; %% full power full arm
rollRelTorque = 4*mm1+2*mm2;
% pitch
mm3 = (sin(60*pi/180))/6*arm_length; %full power reduced arm
pitchRelTorque = 4*mm3/cos(30*pi/180); %increased power on 4 motors
% yaw
% Relative drag each motor - transfer from N to torque in Nm
relDrag = hoverDrag/trustHoover;
% Linearized model transfer functions
% Roll velocity (deg/s) from motor (delta) voltage
Groll  = rollRelTorque/IRoll * tf(1,[1 0]) * Gtr * 180/pi;
% Pitch velociy (deg/s) from motor (delta) voltage
Gpitch = pitchRelTorque/IPitch * tf(1,[1 0]) * Gtr * 180/pi;
% Yaw velocity (deg/s)  from motor (delta) voltage
Gyaw   = relDrag/IYaw * tf(1,[1 0]) * Gtr * 180/pi;
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% design ROLL velocity regulator
rgm = 60;
ral = 0.5;
rNi = 0;
[rw, rkp, rtd] = findpd(Groll, rgm, ral)
% extra filter pole as in Lead to reduce noise
rrlp = rtd*ral;
Gratd= 1; % tf([1],[rrlp/20 1]);
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'rollVel', Groll*Gratd, ...
           rw, rkp, rti, rNi, rtd, ral, rgm, ...
           'Roll velocity control');
%% ROLL controller
Crd = tf([rtd 1],[rtd*ral 1]);
Grvcl = minreal(rkp*Groll*Gratd/(1+rkp*Crd*Groll*Gratd));
Grv2r = tf(1,[1 0]);
Gr = Grvcl*Grv2r;
% design roll angle regulator
raal = 0.2;
ragm = 80;
raNi = 0;
[raw, rakp, ratd] = findpd(Gr, ragm, raal)
Crad = tf([ratd 1],[ratd*raal 1]);
Graol = rakp*Crad*Gr;
Gracl = minreal(rakp*Gr/(1+ Graol), 0.005);
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'roll', Gr, ...
           raw, rakp, rati, raNi, ratd, raal, ragm, ...
           'Roll angle control');
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PITCH control
%% design pitch velocity regulator
pgm = 60;
pal = 0.5;
pNi = 0;
[pw, pkp, ptd] = findpd(Gpitch, pgm, pal)
% extra filter pole as in Lead to reduce noise
pplp = ptd*pal;
Gpatd=1 % tf([1],[pplp 1]);
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'pitchVel', Gpitch*Gpatd, ...
           pw, pkp, pti, pNi, ptd, pal, pgm, ...
           'Pitch velocity control');
%
%% PITCH angle controller
Cpd = tf([ptd 1],[ptd*pal 1]);
Gpvcl = minreal(pkp*Gpitch*Gpatd/(1+pkp*Cpd*Gpitch*Gpatd));
Gpv2p = tf(1,[1 0]);
Gp = Gpvcl*Gpv2p;
% design pitch angle regulator
paal = 0.2;
pagm = 80;
paNi = 0;
[paw, pakp, patd] = findpd(Gp, pagm, paal)
Cpad = tf([patd 1],[patd*paal 1]);
Gpaol = pakp*Cpad*Gp;
Gpacl = minreal(pakp*Gp/(1+Gpaol));
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'pitch', Gp, ...
           paw, pakp, pati, paNi, patd, paal, pagm, ...
           'Pitch angle control');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% YAW controller
%% design yaw velocity regulator
ygm = 60;
yNi = 3;
yal = 0.5;
w = logspace(-1,2,1000);
% integrator limit
yiLim = 500; 
%% additional low pass filter
Gylp = tf([1],[0.0125 1]);
ytd = 0;
yti = 1000;
[yw, ykp, yti, ytd] = findpid(Gyaw*Gylp, ygm, yNi, yal, w)
%[yw, ykp, ytd] = findpd(Gyaw*Gylp, ygm, yal, w)
%[yw, ykp] = findp(Gyaw*Gylp, ygm, w)
%ykp = 500
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
[C, Cd] = showResult(debugPlot,resultFile,filename, 'yawVel', Gyaw*Gylp, ...
           yw, ykp, yti, yNi, ytd, yal, ygm, ...
           'Yaw angle velocity control');
Gol = C*Cd*Gyaw*Gylp;
Yol_poles= pole(Gol)
Yol_zeros= zero(Gol)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The rest is not used in real flight controller
if 0
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% transfer function for next controller 
%Cyi = tf([yti 1],[yti 0]);
Cyd = tf([ytd 1],[ytd*yal 1]);
Cy = ykp;
% closed loop for velocity control
Gyvcl = minreal(Cy*Gyaw*Gylp/(1+Cy*Cyd*Gyaw));
Gyv2p = tf(1,[1 0]);
Gya = Gyvcl*Gyv2p;
%% design yaw angle regulator
yaal = 0.5;
yagm = 50;
yaNi = 4;
%[yaw, yakp, yatd] = findpd(Gya, yagm, yaal)
[yaw, yakp, yati, yatd] = findpid(Gya, yagm, yaNi, yaal)
Cyad = tf([yatd 1],[yatd*yaal 1]);
Gyaol = yakp*Cyad*Gya;
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'yaw', Gya, ...
           yaw, yakp, yati, yaNi, yatd, yaal, yagm, ...
           'Yaw velocity ref to yaw angle');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Y-velocity control (using roll angle in w coordinates)
% estimate transfer function
Gyv = Gracl * tf([1],[1 0]) * trustHoover;
%% Y-velocity controller
yvgm = 70;
yval = 0.3;
yvNi = 0;
[yvw yvkp yvtd] = findpd(Gyv, yvgm, yval)
Cyvd = tf([yvtd 1],[yvtd*yval 1]);
Gyvcl = minreal(yvkp * Gyv / (1 + yvkp*Cyvd*Gyv));
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'y_vel', Gyv, ...
           yvw, yvkp, yvti, yvNi, yvtd, yval, yvgm, ...
           'Roll ref to Y-velocity');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% X-velocity control (using pitch angle)
% estimate transfer function
Gxv = Gpacl * tf([1],[1 0]) * trustHoover;
%% X-velocity controller
xvgm = 70;
xval = 0.3;
xvNi = 0;
[xvw xvkp xvtd] = findpd(Gxv, xvgm, xval)
Cxvd = tf([xvtd 1],[xvtd*xval 1]);
Gxvcl = minreal(xvkp * Gxv / (1 + xvkp*Cxvd*Gxv));
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'x_vel', Gxv, ...
           xvw, xvkp, xvti, xvNi, xvtd, xval, xvgm, ...
           'Pitch ref to X-velocity');
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end