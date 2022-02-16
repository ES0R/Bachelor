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
propellerInertia = 0.016/3*propellerRadius^2;
propellerMass = 0.016 + 3*motorInertia/propellerRadius^2; % scaled up with motor inertia
%% battery
cells = 5; % used by input voltage limiter
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
KDrag = (trust(3,1)-0.3)*Km / (trust(3,3)/60*2*pi)^4;
%% trust constant (Ktrust ~= 5.25e-6 N s^2) proportional to w^2
Ktrust = trust(1,2)/1000/(trust(1,3)/60*2*pi)^2;
%
%% operating point calculation (0.5N) ~4 V på motor
% hover calculation
trustPoint = 0.5; % Newton
hoverVel = sqrt(trustPoint / Ktrust); % in radians/sec
hoverRPM = hoverVel/(2*pi) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel^4; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;

%%
model = 'single_motor';
io = getlinio(model);
% Specify the operating point
% Use the snapshot times
op = 1;
% Linearize the model
sys = linearize(model,io,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
G05 = tf(num, den)
%% operating point calculation (3.5N) ~20V på motor
% hover calculation
trustPoint = 3.5; % Newton
hoverVel = sqrt(trustPoint / Ktrust); % in radians/sec
hoverRPM = hoverVel/(2*pi) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel^4; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;

%%
model = 'single_motor';
io = getlinio(model);
% Specify the operating point
% Use the snapshot times
op = 1;
% Linearize the model
sys = linearize(model,io,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
G25 = tf(num, den)
%% bode
w = logspace(0,2,1000);
figure(2000)
hold off
bode(G05,w)
hold on
bode(G25,w)
grid on
title('from motor voltage to Motor trust');
p05 = pole(G05)
p25= pole(G25)
legend('trust=0.5N', 'trust=2.5N')