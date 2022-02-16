%% parameters for control experiments
close all
clear
%
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
% The 3508-700KV MultiStar is almost too nice to fly! 
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
Kv = 700; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
Ra = 0.083; % ohm
motorMass = 0.102; % motor mass
motorInertia = (0.0422/2)^2*motorMass*0.6; % ~60% is rotating
propellerRadius = 13.5*2.54/2/100;
propellerMassRaw = 0.017;
% mass scaled up with motor inertia
propellerMass = propellerMassRaw + 3*motorInertia/propellerRadius^2; 
%% battery
cells = 4;
batVolt = 3.7*cells; % assumed battery voltage 3.7V per cell

%% propel 14x47 (14" diameter, 4.7" inch pitch (distance for 1 rotation))
% trust table - measure by someone (https://cdn-global-hk.hobbyking.com/media/file/109316975X745660X20.jpg)
% 1 is current (A)
% 2 is trust (g)
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
if 0
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
%% drag konst (KDrag ~= 3.2e-12 Nm s^4) proportional to w^4
deadCurrent = 0.3; % Amps (tomgangsstrøm (statisk friktion?)
KDrag = (trust(3,1)-deadCurrent)*Km / (trust(3,3)/60*2*pi)^4;
% based on 13" propeller measurements
KDrag = 1.5e-9;
%% trust constant (Ktrust ~= 5.25e-6 N s^2) proportional to w^2
Ktrust = trust(1,2)/1000/(trust(1,3)/60*2*pi)^2;
% based on 13" propeller measurements
Ktrust = 0.027;
%% hover 
heightRef = 1.0;
% sample time
Ts = 0.002; % måling interval (sampletime) - sek
%% drone konstants
motorCount = 6;
bodyMass = 0.2; % drone weight (with no motors) [kg]
droneMass = bodyMass + motorCount*motorMass + propellerMassRaw; % kg
batteryMass = 0.1;
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
% arm length (from center to motor, assumed circular)
arm_length = 0.35;
%%%
%% temp controller values to allow simulation
% hover
htd = 1.5;
hal = 0.1;
hkp = 2;
hti = 1.5; % no I
rkp = 0;
rtd = 1;
rti = 1;
ral = 1;
pkp = 0;
ptd = 1;
pal = 1;
pti = 1;
ykp = 0;
ytd = 1;
yti = 1;
yal = 1;
rakp = 0;
ratd = 1;
raal = 1;
rati = 100;
pakp=0;
patd = 1;
paal = 1;
pati = 100;

%% linear analysis - motor to trust
model = 'hexacopter_sim';
load_system(model);
open_system(model);
ios(1) = linio('hexacopter_sim/trust_step',1,'openinput');
ios(2) = linio('hexacopter_sim/hardware',2,'openoutput');
setlinio(model,ios);
% Use the snapshot times
op = 10;
% Linearize the model
sys = linearize(model,ios,op);
%% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gtr = minreal(tf(num, den), 0.005)
trust2height = tf(1,[totalMass 0 0]);
Gh = Gtr * trust2height
%% bodeplot
figure(300)
hold off
w = logspace(-1,2,1000);
bode(Gtr,w)
hold on
bode(Gh,w);
grid on
legend('to trust','to height')
title(' trust voltage to trust sum')
%% design controller
hgm = 30;
hal = 0.07;
hNi = 4;
[hw, hkp, hti, htd] = findpid(Gh, hgm, hNi, hal)
%[hw, hkp, htd] = findpd(Gh, hgm, hal)
%% test
Chd = tf([htd 1],[htd*hal 1]);
Chi = tf([hti 1],[hti 0]);
Ch = Chd*hkp*Chi;
Ghcl = minreal(hkp*Gh*Chi/(1 + Ch*Gh*hkp))
% bode
figure(330)
hold off
bode(Ghcl,w)
hold on
margin(Ch*Gh)
%% roll control identification
%% linear analysis - roll ind to roll velocity
model = 'hexacopter_sim';
load_system(model);
open_system(model);
ior(1) = linio('hexacopter_sim/roll_in',1,'openinput');
ior(2) = linio('hexacopter_sim/platform',10,'openoutput');
setlinio(model,ior);
% Use the snapshot times
op = 10;
% Linearize the model
sysRoll = linearize(model,ior,op);
%% get transfer function
[numR,denR] = ss2tf(sysRoll.A, sysRoll.B, sysRoll.C, sysRoll.D);
Groll = minreal(tf(numR, denR), 0.005)
% bodeplot roll
figure(400)
hold off
bode(Groll)
grid on
title('roll motor volt to roll velocity')
%% design roll velocity regulator
rgm = 80;
ral = 0.2;
[rw, rkp, rtd] = findpd(Groll, rgm, ral)
%% controller
Crd = tf([rtd 1],[rtd*ral 1]);
Grvcl = minreal(rkp*Groll/(1+rkp*Crd*Groll));
Grv2r = tf(1,[1 0]);
Gr = Grvcl*Grv2r;
% design roll angle regulator
raal = 0.2;
ragm = 70;
[raw, rakp, ratd] = findpd(Gr, ragm, raal)
Crad = tf([ratd 1],[ratd*raal 1]);
Graol = rakp*Crad*Gr;
% bode open og closed
figure(500)
hold off
bode(Gr);
hold on
margin(Graol);
bode(Grvcl)
grid on
title('Roll angle');
legend('system','open loop','roll vel closed loop')
%
%% linear analysis - PITCH ind to pitch velocity
model = 'hexacopter_sim';
load_system(model);
open_system(model);
iop(1) = linio('hexacopter_sim/pitch_in',1,'openinput');
iop(2) = linio('hexacopter_sim/platform',11,'openoutput');
setlinio(model,iop);
% Use the snapshot times
op = 9;
% Linearize the model
sysP = linearize(model,iop,op);
%% get transfer function
[numP,denP] = ss2tf(sysP.A, sysP.B, sysP.C, sysP.D);
Gpitch = minreal(tf(numP, denP), 0.005)
% bodeplot roll
figure(510)
hold off
bode(Groll)
hold on
bode(Gpitch)
grid on
title('roll and pitch motor volt to roll/pitch velocity')
legend('roll','pitch')
%% design roll velocity regulator
pgm = 80;
pal = 0.2;
[pw, pkp, ptd] = findpd(Gpitch, pgm, pal)
%% controller
Cpd = tf([ptd 1],[ptd*pal 1]);
Gpvcl = minreal(pkp*Gpitch/(1+pkp*Cpd*Gpitch));
Gpv2p = tf(1,[1 0]);
Gp = Gpvcl*Gpv2p;
% design roll angle regulator
paal = 0.2;
pagm = 70;
[paw, pakp, patd] = findpd(Gp, pagm, paal)
Cpad = tf([patd 1],[patd*paal 1]);
Gpaol = pakp*Cpad*Gp;
% bode open og closed
figure(520)
hold off
bode(Gp);
hold on
margin(Gpaol);
bode(Gpvcl)
grid on
%title('Pitch angle');
legend('system','open loop','pitch vel closed loop')
%
%% linear analysis - YAW ind to yaw velocity
model = 'hexacopter_sim';
load_system(model);
open_system(model);
iop(1) = linio('hexacopter_sim/yaw_in',1,'openinput');
iop(2) = linio('hexacopter_sim/platform',12,'openoutput');
setlinio(model,iop);
% Use the snapshot times
op = 9;
% Linearize the model
sysY = linearize(model,iop,op);
%% get transfer function
[numY,denY] = ss2tf(sysY.A, sysY.B, sysY.C, sysY.D);
Gyaw = minreal(tf(numY, denY), 0.005)
% bodeplot roll
figure(550)
hold off
bode(Groll)
hold on
bode(Gyaw)
grid on
title('roll and yaw volt to roll/yaw velocity')
legend('roll','yaw')
%% design yaw velocity regulator
% first order TF, so set w
ygm = 75;
yNi = 1.5;
w = logspace(1,3,1000);
[yw, ykp, yti] = findpi(Gyaw, ygm, yNi,w)
%%
Cyi = tf([yti 1],[yti 0])
Cy = ykp*Cyi;
%% controller
Gyvcl = minreal(Cy*Gyaw/(1+Cy*Gyaw));
Gyv2p = tf(1,[1 0]);
Gya = Gyvcl*Gyv2p;
figure(565)
hold off
margin(Gyaw*Cy)
hold on
bode(Gyvcl)
bode(Gya)
legend('yaw open loop vel', 'yaw vel closed loop','yaw angle open loop')
grid on
%% design yaw angle regulator
yaal = 0.5;
yagm = 70;
[yaw, yakp, yatd] = findpd(Gya, yagm, yaal)
Cyad = tf([yatd 1],[yatd*yaal 1]);
Gyaol = yakp*Cyad*Gya;
% bode open og closed
figure(570)
hold off
bode(Gya);
hold on
margin(Gyaol);
bode(Gyvcl)
grid on
%title('Pitch angle');
legend('yaw system','yaw open loop','yaw vel closed loop')


