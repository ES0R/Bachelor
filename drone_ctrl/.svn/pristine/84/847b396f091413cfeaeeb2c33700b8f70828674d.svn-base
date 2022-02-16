%% - plot static trust, current and RPM
% trust using a standard scale 25kg max, 1g resolution
close all
clear
%% Trust measurements
% Measurement from esc,motor,propeller test
% data log for sensor has 2000 values (interval=0.01ms)
% 1   Time (ms)
% 2   Control reference input (setpoint) (r)
% 3   after pre-filter (r2)
% 4   error after Kp (ep)
% 5   error after Kp and lead/lag, before I (up)
% 6   integrator output (ui)
% 7   Controller output (u) after limiter
% 8   measurement (m)
% 9   measurement after filter (m2)
% 10  Feed forward output (uf)
data000 = load('log_pitch_angle_vel.txt');
data001 = load('angle_vel_01.txt');
data002 = load('angle_vel_02.txt');
cl001 = load('closed__loop_001.txt');
cl002 = load('closed__loop_002.txt');
cl003 = load('closed__loop_003.txt');
cl004 = load('closed__loop_004.txt');
cl005 = load('closed__loop_005.txt');
cl006 = load('closed__loop_006.txt');
cl007 = load('closed__loop_007.txt');
%%
dd = cl007;
fig = 17000;
%% plot
figure(fig)
hold off
time = dd(:,1) - dd(1,1);
plot(time, dd(:,3))
hold on
plot(time, dd(:,8));
%plot(time, dd(:,9));
% yyaxis right
% plot(time, dd(:,4));
% plot(time, dd(:,5));
legend('ref','m','m after filter')
grid on

%% plot
figure(fig + 2)
hold off
time = dd(:,1) - dd(1,1);
plot(time, dd(:,4))
hold on
plot(time, dd(:,5));
plot(time, dd(:,7));
legend('efter Kp','efter kp og filter','after limiter')
grid on

%% Plot u and m
figure(fig+10)
hold off
yyaxis left
plot(dd(:,8))
ylabel('Measure angle deg')
hold on
yyaxis right
plot(dd(:,7))
ylabel('control signal 0..1000')
grid on
%% estimate of tf from
%% u (+/- 200) motor offset signal
ddu = dd(:,7);
ddpitch = dd(:,8);
ddi = iddata(ddpitch, ddu, 0.0025);
Gpitch20 = tfest(ddi, 2,0)
Gpitch30 = tfest(ddi, 3,0)
Gpitch40 = tfest(ddi, 4,0)
Gpitch31 = tfest(ddi, 3,1)
Gpitch42 = tfest(ddi, 4,2)
Gpitch52 = tfest(ddi, 5,2)

pole20 = pole(Gpitch20)
pole30 = pole(Gpitch30)
pole40 = pole(Gpitch40)
pole31 = pole(Gpitch31)
pole42= pole(Gpitch42)
pole52= pole(Gpitch52)

%% compare result
figure(142)
compare(ddi, Gpitch42)
grid on

figure(131)
compare(ddi, Gpitch31)
grid on

figure(130)
compare(ddi, Gpitch30)
grid on

figure(520)
compare(ddi, Gpitch52)
grid on


%% Bode

h = figure(300)
bode(Gpitch20)
hold on
bode(Gpitch30)
grid on
legend('2 poles', '3 poles')
saveas(h, 'pitch_tfest_01.png')
