%% plot drone data
close all
clear;
%% load data
%
% data log for timing, has 2000 values (interval=2.5 ms)
% data log for timing, has 1250 values (interval=5 ms)
% 1     Log sample time (ms)
% 2     ADC time (ms)
% 3     ADC samples per cycle
% 4     Cyccle time (ms)
% 5     Sensor filter time (ms)
% 6     Control time (ms)
% 7     Cycle process time (ms)
% 8     Cycle start time (sec)
% 9     Get IMU data time (ms)
% 10    Board-coord_conv_time (ms)
% 11    To end filter time (ms)
% 12    gyro (roll) deg/s
% 13    Acc (x) m/s^2
% 14    Gyro reads this sample
% 15    Acc/mag reads this sample
% 16    Pressure reads this sample
% 17    getAccMagData (ms)
%
data5 = load('log_timing_05.txt');
data6 = load('log_timing_06.txt');
data7 = load('log_timing_07.txt');
%
%% plot sample time
dd = data7;
figure(100)
plot(dd(:,1)/1000, dd(:,4));
title('sample time')
grid on
%% plot data bits
figure(100)
hold off
plot(dd(:,1)/1000, dd(:,14));
hold on
plot(dd(:,1)/1000, dd(:,15)*0.6);
plot(dd(:,1)/1000, dd(:,16)*0.5);
title('sample valid')
legend('gyro','acc','pressure')
grid on
%% plot imu read
figure(200)
hold off
plot(dd(1:end,1)/1000, dd(:,9));
hold on
%plot(dd(:,1)/1000, dd(:,7));
plot(dd(:,1)/1000, dd(:,11),'-v');
title('IMU read time')
legend('get IMU data','gyro')
ylabel('ms');
xlabel('sec')
grid on
%% plot Mathword filter time
figure(300)
hold off
plot(dd(:,1)/1000, dd(:,10) - dd(:,9));
hold on
plot(dd(:,1)/1000, dd(:,6));
title('IMU filter time')
legend('IMU filter','Control time')
ylabel('milli sec')
xlabel('sec')
grid on
