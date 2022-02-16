% show data from drone motortest
close all 
clear
%% load data
% data log for sensor has 3125 values (interval=5.0ms)
% data format:
% 1   Time (us)
% 2,3 (a,b) calculated rotations per second
% 3   ESC PW (ms)
% 4   Battery voltage (V)
% 5   Battery current (A)
% 6   Motor temp (deg C)
% 7   ESC temp (deg C)
data_101 = load('log_step_100_300_01.txt');
data_102 = load('log_step_200_500_02.txt');
%%
dd = data_102;
fig = 102;
%%
figure(fig)
t = (dd(:,1)-dd(1,1))/1000000;
hold off
plot(t, dd(:,2))
grid on
hold on
plot(t, dd(:,6))
plot(t, dd(:,4)*10);
legend('RPS ', 'Current (A)', 'PWM (ms)');
 
%%
% timing
figure(fig+1)
hold off
plot(diff(t)*1000)
grid on