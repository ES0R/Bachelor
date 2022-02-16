%% plot drone data
close all
clear;
%% load data
%
% acc    offset (0.031898, -0.029124, 0.030866)
% gyro   offset (1.680432, -0.517817, 0.232171)
% height offset (-4.534304)
% data format:
% 1      Time (ms)
% 2-4    Acc data (m/s2)
% 5-7    Gyro data (deg/s)
% 8      Altitude filtered (m)
% 9      Altitude raw (m)
% 10     Altitude offset (m)
% 11     Altitude ultrasound (m)
% 12     Altitude velocity (m/s)
% 13-15  Roll, pitch, Yaw (radians)
% 16     Temperature (deg C)
% 17     calibration sample
% 
data1 = load('log_imu_hex_001.txt');
%
%% plot acc
dd = data1;
figure(100)
hold off
plot(dd(:,1)/1000, dd(:,2));
hold on
plot(dd(:,1)/1000, dd(:,3));
plot(dd(:,1)/1000, dd(:,4));
title('acc')
legend('acc x','acc y','acc z')
grid on
%% plot gyro
dd = data1;
figure(200)
hold off
plot(dd(:,1)/1000, dd(:,5));
hold on
plot(dd(:,1)/1000, dd(:,6));
plot(dd(:,1)/1000, dd(:,7));
title('gyro')
legend('x','y','z')
grid on
