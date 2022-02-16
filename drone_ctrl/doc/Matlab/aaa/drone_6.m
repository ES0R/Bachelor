%% parameters for control experiments
%% relates to drone_sim.slx model
close all
clear
%% should debug plots be generated
debugPlot = 1; % set to 0 for no plot
% should result be saved in text-file
resultFile = 1; % set to 0 if not desired
filename = 'drone_design.txt';
% simulink model
%% simulink model name
model = 'drone_sim';
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
motorMass = 0.102; % motor mass
motorInertia = (0.0422/2)^2*motorMass*0.6; % ~60% is rotating
propellerRadius = 13.5*2.54/2/100; % meters
propellerMassRaw = 0.017;
deadCurrent = 0.3;
% mass scaled up with motor inertia
propellerMass = propellerMassRaw + 3*motorInertia/propellerRadius^2; 
%% battery
cells = 3;
batVolt = 3.7*cells; % assumed battery voltage 3.7V per cell
%% drone visuals
footX = 0.14; % forward 
footY = 0.17; % width
footZ = 0.14; % height to bars
legX = footX/2;
legY = footY/2;
legZ = footZ/2;
footDiag = hypot(footX,footY);
legLen = sqrt(footX^2 + footY^2 + footZ^2);
footRot = -atan2(footDiag, footZ);
hipX = 0.07;
hipY = 0.13;
hipZ = 0.03;
body = [0.02 0.06 0.15];
batt = [0.15 0.02 0.05]; % approx 4.4Ah
arms = [0.02 0.30 0.02];
motorY = 0.37;
%% drone konstants
motorCount = 6;
bodyMass = 0.3; % drone weight (with no motors) [kg]
droneMass = bodyMass + motorCount*(motorMass + propellerMassRaw); % kg
batteryMass = 0.5; % single battery
batteryDistanceFromCOG = -0.1;
% arm length (from center to motor, assumed circular)
arm_length = motorY;
%% Servo
% servo rotates 180 deg for a 1000us change in PWM
% propeller rotates 20 deg when servo rotates 45 deg
% result in radians
servoUs2Rad = 20/45 * 180/1000 * pi/180;
%% limitations
roll_act_lim = 500;
pitch_act_lim = 500;
yaw_act_lim = 500;
trust_lim = 1000*0.95;
roll_limit = 10; % roll and pitch angle limit (deg)
yaw_rate_limit = 90; % yaw angle rate limit (deg/s)
minimumMotorVolt = 1.0; % lower limit to motor voltage (not to stop)
%heightVelocityLimit = 2; % m/s
%% save results to file
if resultFile
    fileID=fopen(filename,'w');
    fprintf(fileID, '# Result of drone configuration\n');
    fprintf(fileID, '# from mobotware/drone_ctrl/trunk/doc/Matlab/drone_2x2.m\n');
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
    fprintf(fileID, '# Drone mass exclusive of battery (kg)\n');
    fprintf(fileID, 'droneMass %g\n', droneMass);
    fprintf(fileID, '#\n');
    fprintf(fileID, '[motor]\n');
    fprintf(fileID, 'motor %s\n', motor);
    fprintf(fileID, 'resistance %g\n', Ra);
    fprintf(fileID, 'kv %g\n', Kv);
    fprintf(fileID, '# mass in kg\n');
    fprintf(fileID, 'mass %g\n', motorMass);
    fprintf(fileID, 'motorCount %g\n', motorCount);
    fprintf(fileID, '# 2x2 configuration\n');
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
totalMass = droneMass + batteryMass*2;
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
%% undercarrage control
UKp = 800;   % Kp
UKp0 = UKp;  % saved value, as needed to be zero during linearize
pawHgt = 0.02; % base height of foot (paw)
UTau_h = 0.2; % Lead tau
ULimit = 400; % max force
%% append calculated base data
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
hvkp = 50;
hval = 1;
hvtd = 1;
hvti = 1;
hviuse = 0; % use integrator
%
h1td = 1; % tau_d
h1al = 1; % alpha
h1kp = 0;   % K_P
h1ti = 1000; % tau_i
h1iuse = 0;  % use integrator

% roll
rkp = 0;
rtd = 1;
rti = 1;
ral = 1;
% pitch
pvkp = 0;
pvtd = 1;
pval = 1;
pvti = 1;
pv_iuse = 0;
% yaw
yawvkp = 0;
yawvti = 1;
yawvtd = 0;
yawval = 1;
yawvtiUse = 0;
yaw_limit = 1;
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
yatiUse = 0;
% velocity + position x
xvkp = 0;
xvtd = 1;
xval = 1;
xvti = 100;
xvtiUse = 0;
xkp = 0;
xtd = 1;
xal = 1;
xti = 100;
xtiUse = 0;
% velocity + position y
yvkp = 0;
yvtd = 1;
yval = 1;
yvti = 100;
yiLim = 300;
yvtiUse = 0;
% y position
ykp = 0;
ytd = 1;
yti = 1;
yal = 1;
ytiUse = 0;
% mision cycle time
cycleTime = 10;
% disable push  ^ 
pushEnabled = 0;
heightInRef.time=[1,1,18,18]';
% fixed height of 1m for calibration
heightInRef.signals.values=[0,1,1,0.9]';
% push
pushOn = 0;
%% linear analysis - motor to height velocity (including vel filter)
%model='drone2x2_min'
UKp = 0;
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/trust_in',1,'openinput');
ios(2) = linio('drone_sim/drone/height_control/heightVelLp',1,'openoutput');
setlinio(model,ios);
% Use the snapshot time(s) op=[2] for (2 seconds)
op = [2];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Ghv = minreal(tf(num, den), 0.005)
end
%% design height velocity controller
hvgm = 60;
hval = 0.2;
hvNi = 3;
%[hvw, hvkp, hvti, hvtd] = findpid(Ghv, hvgm, hvNi, hval)
[hvw, hvkp, hvti, hvtd] = findpid(Ghv, hvgm,  hvNi, hval)
% result:
% h1 Kp = 100, ti=0.55, td = 0.41
%% debug and save result
showResult(debugPlot,resultFile,filename, 'height_vel_ctrl', Ghv, ...
           hvw, hvkp, hvti, hvNi, hvtd, hval, ...
           hvgm, 'Height velocity controller');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% linear analysis - height vel ref to height
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/height_control/height ctrl1',1,'openinput');
ios(2) = linio('drone_sim/drone/height_control/h-filt-out',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times 2 and 10 seconds
op = [1.2];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
% figure(100)
hold off
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gh1 = minreal(tf(num, den), 0.005)
%   bode(Gh1);
%   hold on
end
% grid on
%% design height controller
h1gm = 75;
h1al = 0.15;
h1Ni = 0;
h1iuse=0;
%[hvw, hvkp, hvti, hvtd] = findpid(Ghv, hvgm, hvNi, hval)
[h1w, h1kp, h1td] = findpd(Gh1, h1gm,  h1al)
% result:
% h1 Kp = 100, ti=0.55, td = 0.41
%% debug and save result
showResult(debugPlot,resultFile,filename, 'height_1_ctrl', Gh1, ...
           h1w, h1kp, h1ti, h1Ni, h1td, h1al, ...
           h1gm, 'height controller');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% design ROLL velocity regulator
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/roll_in',1,'openinput');
ios(2) = linio('drone_sim/drone/roll-vel-deg',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [2.5];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gxrv = minreal(tf(num, den), 0.005)
end
%% roll velocity controller
rgm = 65;
ral = 0.5;
rNi = 0;
[rw, rkp, rtd] = findpd(Gxrv, rgm, ral)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'rollVel', Gxrv, ...
           rw, rkp, rti, rNi, rtd, ral, rgm, ...
           'Roll velocity controller');
%% ROLL controller
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/roll ctrl',1,'openinput');
ios(2) = linio('drone_sim/drone/roll-deg',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [3];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hover
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gxr = minreal(tf(num, den), 0.005)
end
%% roll controller
raal = 0.25;
ragm = 70;
raNi = 0;
[raw, rakp, ratd] = findpd(Gxr, ragm, raal)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'roll', Gxr, ...
           raw, rakp, rati, raNi, ratd, raal, ragm, ...
           'Roll angle controller');
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PITCH control - velocity
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/u_pitch',1,'openinput');
ios(2) = linio('drone_sim/drone/pitch-vel-deg',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [3];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gyrv = minreal(tf(num, den), 0.005)
end
% figure(200)
% bode(Gyrv)
% grid on
%% design pitch velocity regulator
pvgm = 70;
pval = 0.4;
pvNi = 0;
[pvw, pvkp, pvtd] = findpd(Gyrv, pvgm,pval)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'pitchVel', Gyrv, ...
           pvw, pvkp, pvti, pvNi, pvtd, pval, pvgm, ...
           'Pitch velocity controller');
%
%% PITCH angle controller
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/pitch ctrl',1,'openinput');
ios(2) = linio('drone_sim/drone/pitch-deg',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [3];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% figure(100)
w = logspace(-1,3,1000);
% hold off
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gyra = minreal(tf(num, den), 0.005)
%   bode(Gyra,w);
%   hold on
end
% grid on

%% design pitch angle regulator
paal = 0.2;
pagm = 50;
paNi = 0;
[paw, pakp, patd] = findpd(Gyra, pagm, paal)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'pitch', Gyra, ...
           paw, pakp, pati, paNi, patd, paal, pagm, ...
           'Pitch angle controller');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% YAW controller
%% design yaw velocity regulator
UKp = 0; % remove undercarrage control
load_system(model);
open_system(model);
ios(1) = linio('drone_sim/drone/u_yaw',1,'openinput');
ios(2) = linio('drone_sim/drone/yaw-vel-deg',1,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = [3];
% Linearize the model
sys = linearize(model,ios,op);
%
UKp = UKp0;
%% get transfer function
% figure(100)
w = logspace(-1,3,1000);
% hold off
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gzra = minreal(tf(num, den), 0.005)
%   bode(Gzra,w);
%   hold on
end
% grid on
%% design yaw vel ctrl
yawvgm = 70;
yawval = 0.3;
yawvNi = 3.5;
[yawvw, yawvkp, yawvti, yawvtd] = findpid(Gzra, yawvgm, yawvNi, yawval)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
[C, Cd] = showResult(debugPlot,resultFile,filename, 'yawVel', Gzra, ...
           yawvw, yawvkp, yawvti, yawvNi, yawvtd, yawval, yawvgm, ...
           'Yaw angle velocity controller');
%% run simulation
pushOn = 1;
sim('drone_sim')
%%