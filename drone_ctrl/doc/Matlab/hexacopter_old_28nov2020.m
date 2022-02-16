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
%% motorværdier
% EMAX GT2210/09 with 9x4 propeller
if 0
    Kv = 1780; % RPM pr volt
    Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
    %% Motor/propeller data from supplier
    cells = 3;
    batVolt = 3.7*cells; % assumed battery voltage 3.7V per cell
    rpmMax = 3508; % max RPM - assumed full battery voltage over motor
    ampMax = 5; % current at max RPM in amps
    trustMax = 1.6; % trust in kg
    %
    g = 9.80665; % m/s^2 gravity acceleration
    backEMF = rpmMax/Kv; % voltage generated from rotation
    RaVolt = batVolt - backEMF; % remainig voltage over resistance
    Ra = RaVolt/ampMax; % (winding) resistance [Ohm]
    % effect of propeller in trust and drag
    Ktrust = (trustMax * g)/(rpmMax / 60 * 2 * pi)^2; % trust i N/(rad/s)
    % drag at max
    KDrag = ampMax * Km / (rpmMax / 60 * 2 * pi)^4; % drag per rad/sec [Nms]
    motorMass = 0.055; % mass of one motor [kg]
    propellerMass = 0.015; % scaled up to get more realistic inertia
    propellerRadius = 8.5*2.54/2;
end
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
% mass scaled up with motor inertia
propellerMass = propellerMassRaw + 3*motorInertia/propellerRadius^2; 
%% battery
cells = 4;
batVolt = 3.7*cells; % assumed battery voltage 3.7V per cell
%% drone konstants
motorCount = 6;
bodyMass = 0.2; % drone weight (with no motors) [kg]
droneMass = bodyMass + motorCount*(motorMass + propellerMassRaw); % kg
batteryMass = 0.3;
batteryDistanceFromCOG = -0.1;
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
roll_limit = 0.5; % roll and pitch angle limit (radians)
minimumMotorVolt = 3.0; % lower limit to motor voltage (not to stop)
heightVelocityLimit = 2; % m/s
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
    fprintf(fileID, '# max roll and tilt angle limit (radians)\n');
    fprintf(fileID, 'rollLimit %g\n', roll_limit);
    fprintf(fileID, '# minimum motor voltage (at minimum RPM)\n');
    fprintf(fileID, 'minMotorVolt %g\n', minimumMotorVolt);
    fprintf(fileID, '#\n');
    fclose(fileID);
end
%
if 0
    % not used, as it is a wroung propeller (found on internet)
    %% propel 14x47 (14" diameter, 4.7" inch pitch (distance for 1 rotation))
    % trust table - measure by someone (https://cdn-global-hk.hobbyking.com/media/file/109316975X745660X20.jpg)
    % 1 is current (A)
    % 2 is trust g (gram)
    % 3 is RPM
    % 4 is power use (W) (14.8V)
    trust = [1 290 2245 14.8; ...
             2 420 2751 29.6; ...
             3 530 3128 44.4; ...
             4 620 3394 59.2; ...
             5 720 3574 74; ...
             6 810 3797 88.8; ...
             7 890 3951 103.6; ...
             8 960 4122 118.4; ...
             9 1010 4225 133.2; ...
             10 1080 4337 148; ...
             11 1130 4448 162.8; ...
             12 1200 4560 177.6; ...
             14 1300 4722 207.2; ...
             16 1390 4851 236.8; ...
             18 1480 5022 266.4; ...
             20 1550 5134 296; ...
             23 1640 5297 340.4; ...
             27 1780 5485 399.6];
    %%
    % append trust data to file
        if resultFile
            fileID=fopen(filename,'a');
            fprintf(fileID, '# motor and propeller trust data used\n');
            fprintf(fileID, '# 4 columns with\n');
            fprintf(fileID, '#   1 is current (A)\n');
            fprintf(fileID, '#   2 is trust (g)\n');
            fprintf(fileID, '#   3 is RPM\n');
            fprintf(fileID, '#   4 is power use (W) (14.8V)\n');
            fprintf(fileID, '#\n');
            for i = 1:size(trust,1)
              fprintf(fileID, 'trust %g %g %g %g\n', ...
                  trust(i,1), trust(i,2), trust(i,3), trust(i,4));
            end
            fprintf(fileID, '#\n');
            fclose(fileID);
        end
    %% - to seerelation to trust and drag
    if debugPlot
        figure(100)
        hold off
        plot(trust(:,1), trust(:,2));
        hold on
        plot(trust(:,1), trust(:,3)./60*2*pi);
        grid on
        legend('trust (g)','rad/s');
        xlabel('amps (A)')
        title('3508-700KV MultiStar - 14x4.7" prop');
        figure(103)
        hold off
        plot(trust(:,3), trust(:,2)./1000);
        hold on
        plot(trust(:,3), trust(:,1)/10);
        %plot(trust(:,3), trust(:,1).*Km./(trust(:,3).^2/60*2*pi)*100000000);
        grid on
        title('3508-700KV MultiStar - 14x4.7" prop');
        legend('trust (kg)','amps (A/10)');
        xlabel('RPM')
        figure(107)
        plot(trust(:,1)*Km, (trust(:,3)/60*2*pi).^4)
        xlabel('motor torque (Nm)')
        ylabel('(\omega rad/s)^4')
        title('3508-700KV MultiStar - 14x4.7" prop');
        figure(109)
        plot(trust(:,3), (trust(:,1)-0.3)*Km./(trust(:,3)/60*2*pi).^4)
        xlabel('RPM')
        ylabel('Drag konstant [Nm \omega^{-4}]')
        title('3508-700KV MultiStar - 14x4.7" prop');
        figure(112)
        plot(trust(:,3), trust(:,2)/1000./(trust(:,3)/60*2*pi).^2)
        title('3508-700KV MultiStar - 14x4.7" prop');
        xlabel('RPM')
        ylabel('Ktrust [kg \omega^{-2}]')
    end
end
%% drag konst (KDrag ~= 3.2e-12 Nm s^4) proportional to w^4
% deadCurrent = 0.3; % Amps (tomgangsstrøm (statisk friktion?)
% KDrag = (trust(3,1)-deadCurrent)*Km / (trust(3,3)/60*2*pi)^4;
% KDrag = (trust(3,1)-deadCurrent)*Km / (trust(3,3)/60*2*pi)^3;
% Kdrag = 1e-9  % using w^3
% based on 13" propeller measurements (proportional to w^3)
KDrag = 1.5e-9;
%% trust constant (Ktrust ~= 5.25e-6 N s^2) proportional to w^2
%Ktrust = trust(1,2)/1000/(trust(1,3)/60*2*pi)^2;
% Ktrust = 5.2e-6
% based on 13" propeller measurements
Ktrust = 2.7e-5;
%% hover 
heightRef = 1.0;
% sample time
Ts = 0.002; % måling interval (sampletime) - sek
%% hover calculation
g = 9.82;
totalMass = droneMass + batteryMass;
trustHoover = totalMass * g; % Newton
trustPerPropeller = trustHoover / motorCount; % [N]
% hover calculation
hoverVel = sqrt(trustPerPropeller / Ktrust); % in radians/sec
hoverRPM = hoverVel/(2*pi) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel^4; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;
%% height complement filter
h_tau = 1.5;  % filter pole for complement transition
h_beta = 5;   % pole replacing integrator - beta from filter pole
h_cfac = 0.93; % filter pole adjust for post filter only
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
ykp = 0;
ytd = 1;
yal = 1;
yti = 100;
% disable push
pushEnabled = 0;
heightInRef.time=[0,11]';
% fixed height of 1m for calibration
heightInRef.signals.values=[1,1]';
%% linear analysis - motor to trust
load_system(model);
open_system(model);
ios(1) = linio('hexacopter_sim/base_controlled_drone/trust_in',1,'openinput');
ios(2) = linio('hexacopter_sim/base_controlled_drone/hardware',2,'openoutput');
setlinio(model,ios);
% Use the snapshot times 2 and 10 seconds
op = [2,10];
% Linearize the model
sys = linearize(model,ios,op);
%% get transfer function
% transfer function at 2 and 10 seconds
% 10 seconds should be in hoover
for M = 1:size(op,2)
  [num,den] = ss2tf(sys.A(:,:,M), sys.B(:,:,M), sys.C(:,:,M), sys.D(:,:,M));
  Gtr = minreal(tf(num, den), 0.005)
end
% result:
% 2s  = tf([0.84],[1 12.7])
% 10s = tf([1.10],[1 13.5])
%
%% height without height velocity controller
% hover value for take off value
m = bode(Gtr,0); %% static gain
hoverControl = trustHoover / m;
% height control loop
hD = 0.2; % velocity drag (groundless estimate 30m/s => 6N drag)
trust2height1 = tf(1,[totalMass hD 0]);
Gh1 = Gtr * trust2height1
% result
% Gh1 = tf([1.10],[1.13  15.41  2.70  0])
% poles:  s = 0, -13, -0.18
%% design height velocity controller
h1gm = 30;
h1al = 0.1;
h1Ni = 3;
%[hvw, hvkp, hvti, hvtd] = findpid(Ghv, hvgm, hvNi, hval)
[h1w, h1kp, h1ti, h1td] = findpid(Gh1, h1gm,  h1Ni, h1al)
% result:
% h1 Kp = 26.7, ti=1.2, td = 1.26
%% debug and save result
showResult(debugPlot,resultFile,filename, 'height_in_1_ctrl', Gh1, ...
           h1w, h1kp, h1ti, h1Ni, h1td, h1al, ...
           h1gm, 'motor to height');
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
% Roll velocity (rad/s) from motor (delta) voltage
Groll  = rollRelTorque/IRoll * tf(1,[1 0]) * Gtr;
% Pitch velociy (rad/s) from motor (delta) voltage
Gpitch = pitchRelTorque/IPitch * tf(1,[1 0]) * Gtr;
% Yaw velocity (rad/s)  from motor (delta) voltage
Gyaw   = relDrag/IYaw * tf(1,[1 0]) * Gtr;
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% design ROLL velocity regulator
rgm = 70;
ral = 0.2;
rNi = 0;
[rw, rkp, rtd] = findpd(Groll, rgm, ral)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'rollVel', Groll, ...
           rw, rkp, rti, rNi, rtd, ral, rgm, ...
           'volt to roll velocity');
%% ROLL controller
Crd = tf([rtd 1],[rtd*ral 1]);
Grvcl = minreal(rkp*Groll/(1+rkp*Crd*Groll));
Grv2r = tf(1,[1 0]);
Gr = Grvcl*Grv2r;
% design roll angle regulator
raal = 0.1;
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
           'roll vel ref to roll angle');
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PITCH control
%% design pitch velocity regulator
pgm = 70;
pal = 0.2;
pNi = 0;
[pw, pkp, ptd] = findpd(Gpitch, pgm, pal)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'pitchVel', Gpitch, ...
           pw, pkp, pti, pNi, ptd, pal, pgm, ...
           'motor volt to pitch velocity');
%
%% PITCH angle controller
Cpd = tf([ptd 1],[ptd*pal 1]);
Gpvcl = minreal(pkp*Gpitch/(1+pkp*Cpd*Gpitch));
Gpv2p = tf(1,[1 0]);
Gp = Gpvcl*Gpv2p;
% design pitch angle regulator
paal = 0.1;
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
           'pitch vel ref to pitch angle');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% YAW controller
%% design yaw velocity regulator
ygm = 65;
yNi = 0;
yal = 0.5;
w = logspace(0,2,1000);
[yw, ykp, ytd] = findpd(Gyaw, ygm, yal, w)
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'yawVel', Gyaw, ...
           yw, ykp, yti, yNi, ytd, yal, ygm, ...
           'Motor volt to yaw angle velocity');
% 
%% transfer function for next controller
%Cyi = tf([yti 1],[yti 0]);
Cyd = tf([ytd 1],[ytd*yal 1]);
Cy = ykp;
% closed loop for velocity control
Gyvcl = minreal(Cy*Gyaw/(1+Cy*Cyd*Gyaw));
Gyv2p = tf(1,[1 0]);
Gya = Gyvcl*Gyv2p;
%% design yaw angle regulator
yaal = 0.5;
yagm = 70;
yaNi = 0;
[yaw, yakp, yatd] = findpd(Gya, yagm, yaal)
Cyad = tf([yatd 1],[yatd*yaal 1]);
Gyaol = yakp*Cyad*Gya;
%% debug and save result
% showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
showResult(debugPlot,resultFile,filename, 'yaw', Gya, ...
           yaw, yakp, yati, yaNi, yatd, yaal, yagm, ...
           'Yaw velocity ref to yaw angle');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Y-velocity control (using roll angle)
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
%% height complementary filter
