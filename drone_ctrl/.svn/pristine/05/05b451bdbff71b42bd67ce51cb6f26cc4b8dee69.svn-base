%% plot drone data
close all
clear;
%% load data
%
% data format:
% 1     Time (ms)
% 2     Arm state (init, disarmed, armed, fail)
% 3     Flight state (on ground, starting, flight, landing)
% 4     Height ref (trust ref)
% 5     Roll ref (deg)
% 6     Pitch ref (deg)
% 7     Yaw ref (deg/s)
% 8     Height value to mixer (trust ref) o/oo
% 9     Roll value to mixer o/oo
% 10    Pitch value to mixer o/oo
% 11    Yaw value to mixer o/oo
% 12    Height position (m) filtered 
% 13    Roll angle (deg)
% 14    Pitch angle (deg)
% 15    Yaw velocity (deg/s)
% 16    Height velocity (m/s)
% on PSU 14.2V
%
data1 = load('log-pitch_002.txt');
data2 = load('log-pitch_003.txt');
data4 = load('log-pitch_004.txt'); %// psu 16.2V
data6 = load('log-pitch_006.txt'); %   psu 16.2V
%
%% plot height
dd = data6;
figure(100)
plot(dd(:,1)/1000, dd(:,2));
grid on
%%
figure(200)
hold off
plot(dd(:,6))
hold on
plot(dd(:,10)/10)
plot(dd(:,14))
xlabel('sec')
title('pitch')
legend('pitch ref','pitch-mixer','pitch')
%% tfest
%% estimate mixer input to pitch
dd = data6;
N1 = 150;
N2 = 900
ddi = dd(N1:N2,10) - 6;   % start value 0.4 * 4 = 1.6 - or from plot
ddo = dd(N1:N2,14) - 0.8; % offset from plot for right motor
% plot input/output adjusted
figure(271)
hold off
plot(ddi/10)
hold on
plot(ddo)
grid on
% prepater ident
id = iddata(ddo, ddi, 0.0025);
G30 = tfest(id, 3,0)  % 2 poles
%G40 = tfest(id, 4,0)  % 2 poles
G51 = tfest(id, 5,1)  % 
G62 = tfest(id, 6,2)  % 
%% assessment of result
figure(142)
compare(id, G51)
title ('Pitch in mixer to pitch angle in deg (G51)')
grid on
figure(1462)
compare(id, G62)
title ('Pitch in mixer to pitch angle in deg (G62)')
grid on
figure(1463)
compare(id, G30)
title ('Pitch in mixer to pitch angle in deg (G62)')
grid on
%%
figure(143)
hold off
bode(G30)
hold on
bode(G51)
%bode(G62)
legend('G30','G51');
grid on
