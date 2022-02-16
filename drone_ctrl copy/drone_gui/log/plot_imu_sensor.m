% show data from drone motortest
close all 
clear
%% load data
% data log for IMU has 1923 values (interval=10 ms)
% updating using compass=1
% acc    offset (0.017628, -0.141855, 0.027227)
% gyro   offset (1.433748, -0.818761, 0.086480)
% height offset (-62.685501)
% data format:
% 1     Time (ms)
% 2-4   Acc data (m/s2)
% 5-7   Gyro data (deg/s)
% 8     Altitude filtered (m)
% 9     Altitude raw (m)
% 10-12 Roll, pitch, Yaw (degree)
% 13    Temperature (deg C)
data_11 = load('log_imu_alt_11.txt'); 
data_12 = load('log_imu_alt_12.txt'); 
data_13 = load('log_imu_alt_13.txt'); % fudge 16
data_14 = load('log_imu_alt_14.txt'); % fudge 1
data_15 = load('log_imu_alt_15.txt'); % no acc
data_16 = load('log_imu_alt_16.txt'); % no acc - tau=0.5
data_17 = load('log_imu_alt_17.txt'); % tau=2
data_18 = load('log_imu_alt_18.txt'); % tau=0.5 - 0-1.5m - 1m - 1.5m - 0m og retur
data_19 = load('log_imu_alt_19.txt'); % tau=0.5 - slower movement 0m -> 1.5m -> 0m
data_20 = load('log_imu_alt_20.txt'); % tau=2 - beta=5
data_21 = load('log_imu_alt_21.txt'); % tau=0.5 - beta=5
data_22 = load('log_imu_alt_22.txt'); % tau=0.9 - beta=2.1
data_23 = load('log_imu_alt_23.txt'); % tau=1 - beta=5
data_24 = load('log_imu_alt_24.txt'); % tau=1 - beta=10
%% - with new 1st order compl filter
data_30 = load('log_imu_alt_30.txt'); % tau=1 - beta=5
data_31 = load('log_imu_alt_31.txt'); % tau=1 - beta=5

data_40 = load('log_imu_alt_40.txt'); % tau=1 - beta=5

%%
dd = data_30;
fig = 3000;
%% Roll - pitch
h = figure(fig)
di = 2:size(dd,1);
t = dd(di,1)/1000;
hold off
plot(t, dd(di,10))
grid on
hold on
plot(t, dd(di,11))
xlabel('time (sek)')
ylabel('Degrees')
title('Prop Shield data')
legend('roll',...
    'pitch','yaw','location','east')
%saveas(h,'imu_log_01.png')
%% acceleration raw
figure(fig+1)
hold off
plot(dd(:,1), dd(:,2))
hold on
plot(dd(:,1), dd(:,3))
plot(dd(:,1), dd(:,4))
grid on
legend('x','y','z')
mx = mean(dd(:,2))
sx = std(dd(:,2))
my = mean(dd(:,3))
sy = std(dd(:,3))
mz = mean(dd(:,4))
sz = std(dd(:,4))

%%
di = 2:size(dd,1);
hvel = zeros(size(dd,1),1);
hpos = zeros(size(dd,1),1);
for i = di
  hvel(i) = hvel(i-1) + (dd(i,4)-1)*0.01;
  hpos(i) = hpos(i-1) + (hvel(i))*0.01;
end
%% convert acc to world coordinates
accw = zeros(size(dd,1),5);
for i = di
 R = rotmat3x3(dd(i,10), dd(i,11), dd(i,12));
 v = R * [dd(i,2); dd(i,3); dd(i,4)];
 accw(i,1) = v(1);
 accw(i,2) = v(2);
 accw(i,3) = v(3);
 accw(i,4) = sqrt(dd(i,2)^2 + dd(i,3)^2 + dd(i,4)^2);
 accw(i,5) = sqrt(v(1)^2 + v(2)^2 + v(3)^2);
end
%% plot acc amplitude
figure(fig+2);
hold off
plot(dd(:,1), accw(:,4)); % z-acc raw
hold on
grid on
plot(dd(:,1), accw(:,3)); % y-acc (world)

plot(dd(:,1), accw(:,2)); % x-acc world
% plot(dd(:,1), accw(:,1)); % x-acc world
plot(dd(:,1), dd(:,10)); % roll
plot(dd(:,1), dd(:,11)); % pitch
legend('|acc raw|','z-acc w','y-acc w','roll','pitch')
%%
%% altitude
h = figure(fig + 1);
%
t = dd(di,1)/1000;
hold off
plot(t, dd(di,9))
grid on
hold on
plot(t, dd(di,8),'linewidth',2)
plot(t, dd(di,4),'linewidth',1)
plot(t, hvel(di,1))
plot(t, hpos(di,1))
xlabel('time (sek)')
ylabel('m/Degrees')
title('Prop Shield data')
legend('alt raw','Altitude filt (m)', 'acc',...
    'vel','pos','location','south east')
%saveas(h,'imu_log_01.png')
%% Complement filter
tb = 2; % seconds
G1 = tf([1],[tb 1]);
G = G1*G1;
T = 0.0025;
c2d(G,T,'tustin')

%%
function [R] = rotmat3x3(roll, pitch, yaw)
    Rr = [1, 0, 0; ...
          0, cos(roll), -sin(roll); ...
          0, sin(roll), cos(roll)];
    Rp = [cos(pitch), 0, sin(pitch); ...
          0,          1,     0; ...
          -sin(pitch), 0, cos(pitch)];
    Ry = [cos(yaw), -sin(yaw), 0; ...
          sin(yaw), cos(yaw), 0; ...
          0,              0,       1];
    R = Ry*Rp*Rr;
end
