% show data from drone motortest
close all 
clear
%% load data
% data log for sensor has 3125 values (interval=5.0ms)
% data format:
% 1   Time (ms)
% 2,3 (a,b) calculated rotations per second
% 4   ESC PW (ms)
% 5   Battery voltage (V)
% 6   Battery current (A)
% 7   Motor temp (deg C)
% 8   ESC temp (deg C)
% data_101 = load('log_step_100_300_01.txt');
% data_102 = load('log_timing.txt');
% data_103 = load('log_step_200_500_03.txt');
% data_104 = load('log_step_100_300_04.txt');
% data_105 = load('log_step_100_500_05.txt');
% data_106 = load('log_step_100_700_06.txt'); % 18" prop
data_110 = load('log_14x55_100-900x5ms_11v.txt'); % 18" prop
data_111 = load('log_14x55_100-900x5ms_15v.txt'); % 18" prop
data_201 = load('log_sunny_KV300_18x5.5.txt'); % 18" prop
%%
dd = data_201;
fig = 1000;
%%
h = figure(fig)
di = 2:size(dd,1);
t = dd(di,1)/1000;
hold off
plot(t, dd(di,2))
grid on
hold on
% plot(t, dd(di,3))
plot(t, dd(di,4)/10)
plot(t, dd(di,6))
xlabel('time (sek)')
title('3508-700, prop 14x5.5, 11V, time performance')
legend('RPS (Rotations per second)', 'ESC control PW*10 (1-2ms)',...
    'Motor current (Amps)','location','east')
% plot(t, dd(:,6))
% plot(t, dd(:,4)*10);
% legend('RPS ', 'Current (A)', 'PWM (ms)');

%saveas(h,'3508-700-14x5.5-11v.png')
%%
% temperature
figure(fig+1)
hold off
plot(t, dd(di,7))
hold on
plot(t, dd(di,8))
grid on
%% - next section
close all
clear
%% Trust measurements
% Measurement from esc,motor,propeller test
% file created 2020-10-04 17:40:42.147504
% 1: esc value (motor 1) 0=1ms, 1024 = 2ms
% 2: rps (motor 1) a rotations per second)
% 3: rps (motor 1) b rotations per second)
% 4: Motor voltage (volt)
% 5: total current (amps)
% 6: trust force (gram force)
% 7: CCV (rotation direction)
% 8: Temperature motor (deg C)
% 9: Temperature ESC (deg C)
trust001 = load('3508-700_14x5.5.txt');
trust002 = load('3508-700_18x5.5.txt');
trust003 = load('3508-700_13x4.5.txt');
%% motor constant
motor = '3508-700KV';
prop = '14x5.5';
Kv = 700; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
Ra = 0.083; % ohm
%% plot current-trust
trust1 = trust001; %14"
trust2 = trust002; %18"
trust3 = trust003; %13"
vf1 = trust1(:,4);
vf2 = trust2(:,4);
vf3 = trust3(:,4);
h=figure(100)
    hold off
    plot(trust3(:,5).*vf3, trust3(:,6)*9.82/1000,':','linewidth',2);
    hold on
    plot(trust1(:,5).*vf1, trust1(:,6)*9.82/1000,'linewidth',1.5);
    plot(trust2(:,5).*vf2, trust2(:,6)*9.82/1000,'--','linewidth',2);
    plot(trust3(:,5).*vf3, trust3(:,2)*60/1000,':','linewidth',1);
    plot(trust1(:,5).*vf1, trust1(:,2)*60/1000,'linewidth',1);
    plot(trust2(:,5).*vf2, trust2(:,2)*60/1000,'--','linewidth',1);
%     plot(trust1(:,5).*vf1, trust1(:,2)*2*pi*14*0.0254/2);
%     plot(trust2(:,5).*vf2, trust2(:,2)*2*pi*18*0.0254/2,'--');
    grid on
    legend('trust (N) 13"','trust (N) 14"','trust (N) 18"', ...
        'RPM/1000 13"','RPM/1000 14"','RPM/1000 18"',...
        'location','north west');
    xlabel('Motor input power (W)')
    title('3508-700KV MultiStar - 13/14/18" prop');
saveas(h,'trust-per-power-3508-700.png')
%% ESC -- trust
h = figure(102)
    hold off
    plot(trust3(:,1)/1000 + 1, trust3(:,6)*0.0098);
    hold on
    plot(trust1(:,1)/1000 + 1, trust1(:,6)*0.0098);
    plot(trust2(:,1)/1000 + 1, trust2(:,6)*0.0098,'--');
    grid on
    legend('thrust (N) 13"', 'trust (N) 14"', ...
        'thrust (N) 18"','location','north west');
    xlabel('ESC PW (1-2ms) (11V battery)')
    title('3508-700KV MultiStar - transfer gain');
saveas(h, 'trust-TF-3508-700.png')
%% transfer function gain
% 13"
for i = 1:size(trust3,1)-1
    % delta trust
    dN3(i) = trust3(i+1,6) - trust3(i,6);
    % delta ESC input in ms
    dW3(i) = (trust3(i+1,1) - trust3(i,1))/1000;
    % delta rad/s
    dRps31(i) = (trust3(i+1,2)*2*pi) - (trust3(i,2)*2*pi);
    dRps32(i) = (trust3(i+1,2)*2*pi)^2 - (trust3(i,2)*2*pi)^2;
    dRps33(i) = (trust3(i+1,2)*2*pi)^3 - (trust3(i,2)*2*pi)^3;
    % delta motor torque
    dTau3(i) = trust3(i+1,5)*Km - trust3(i,5)*Km;
    % trust per ms.
    dNw3(i) = dN3(i)/dW3(i);
end
%14"
for i = 1:size(trust1,1)-1
    dN1(i) = trust1(i+1,6) - trust1(i,6);
    dW1(i) = (trust1(i+1,1) - trust1(i,1))/1000;
    % delta rad/s
    dRps11(i) = (trust1(i+1,2)*2*pi) - (trust1(i,2)*2*pi);
    dRps12(i) = (trust1(i+1,2)*2*pi)^2 - (trust1(i,2)*2*pi)^2;
    dRps13(i) = (trust1(i+1,2)*2*pi)^3 - (trust1(i,2)*2*pi)^3;
    % delta motor torque
    dTau1(i) = trust1(i+1,5)*Km - trust1(i,5)*Km;
    % trust per ms
    dNw1(i) = dN1(i)/dW1(i);
end
%18"
for i = 1:size(trust2,1)-1
    dN2(i) = trust2(i+1,6) - trust2(i,6);
    dW2(i) = (trust2(i+1,1) - trust2(i,1))/1000;
    % delta rad/s
    dRps21(i) = (trust2(i+1,2)*2*pi) - (trust2(i,2)*2*pi);
    dRps22(i) = (trust2(i+1,2)*2*pi)^2 - (trust2(i,2)*2*pi)^2;
    dRps23(i) = (trust2(i+1,2)*2*pi)^3 - (trust2(i,2)*2*pi)^3;
    % delta motor torque
    dTau2(i) = trust2(i+1,5)*Km - trust2(i,5)*Km;
    % trust per ms
    dNw2(i) = dN2(i)/dW2(i);
end
%% - Transfer gain constant
h = figure(209);
hold off
plot((trust3(2:end,1)/1000-dW3(3)/2), dNw3*0.0098)
hold on
plot((trust1(2:end,1)/1000-dW1(3)/2), dNw1*0.0098,'-x')
plot((trust2(2:end,1)/1000-dW2(3)/2), dNw2*0.0098,'-o')
grid on
xlabel('ESC pulse (ms)')
ylabel('Gain constant (K_0) Newton per ms)')
legend('13" prop','14" prop','18" prop','aaa','location','south')
%saveas(h,'transfer_gain_N-per-ms.png')
%% Transfer gain
h = figure(206)
hold off
plot(trust3(2:end,1)/1000, dNw3*0.0098)
hold on
plot(trust1(2:end,1)/1000, dNw1*0.0098,'-x')
plot(trust2(2:end,1)/1000, dNw2*0.0098,'-o')
grid on
xlabel('ESC pulse (ms)')
ylabel('Transfer Gain (K_0) Newton per ms)')
legend('13" prop','14" prop','18" prop','location','south')
%saveas(h,'transfer_gain_N-per-ms.png')
%% drag constant
% 14"
Kd11 = dTau1./dRps11;
Kd12 = dTau1./(dRps12);
Kd13 = dTau1./(dRps13);
% 18"
Kd21 = dTau2./dRps21;
Kd22 = dTau2./(dRps22);
Kd23 = dTau2./(dRps23);
% 13"
Kd31 = dTau3./dRps31;
Kd32 = dTau3./(dRps32);
Kd33 = dTau3./(dRps33);
% Drag constant (13" and 14")
h = figure(251);
hold off
plot(trust3(2:end,1)/1000 + 1, Kd33);
hold on
plot(trust1(2:end,1)/1000 + 1, Kd13);
%plot(trust2(2:end,1)/1000, Kd23);
xlabel('esc (ms)')
ylabel('drag Constant (\tau/(\omega^3)')
legend('13" propeller','14" propeller','18"','location','northwest')
title('drag constant 3508-700"')
grid on
saveas(h,'3508-700-drag-constant.png')
%% trust constant
Kt1 = dN1./dRps12; % 14"
Kt2 = dN2./dRps22; % 18"
Kt3 = dN3./dRps32; % 13"
h = figure(261);
hold off
plot(trust3(2:end,1)/1000 + 1, Kt3*9.82/1000)
hold on
plot(trust1(2:end,1)/1000 + 1, Kt1*9.82/1000)
%plot(trust2(2:end,1)/1000, Kt2*9.82) % 18"
xlabel('esc (ms)')
ylabel('Trust Constant (N/(\omega^2)')
legend('13" propeller','14" propeller','location','northwest')
title('Trust constant 3508-700"')
grid on
saveas(h,'3508-700-trust-constant.png')

%% total motor torque --- rpm^x
    figure(107)
    hold off
    plot((trust1(:,2)*2*pi).^3, trust1(:,5)*Km)
    hold on
    plot((trust2(:,2)*2*pi).^3,trust2(:,5)*Km, '--')
    plot((trust3(:,2)*2*pi).^3,trust3(:,5)*Km, '--')
    grid on
    ylabel('motor torque (Nm)')
    xlabel('(\omega rad/s)^3')
    legend('13" prop','14" prop','18" prop','location','south')
    title('3508-700KV MultiStar - should look linear');
%% % less relevant graphs
%% RPS -- trust
    figure(106)
    hold off
    plot(trust1(:,2), trust1(:,6)./1000);
    hold on
    plot(trust1(:,2), trust1(:,5)/10);
    plot(trust2(:,2), trust2(:,5)/10,'--');
    plot(trust2(:,2), trust2(:,6)./1000,'--');
    grid on
    title('3508-700KV MultiStar - 14/18x4.7" prop');
    legend('trust (kg) 14"','amps (A/10) 14"', ...
           'trust (kg) 18"','amps (A/10) 18"');
    xlabel('RPS')
%% RPS --- drag constant 14"
    figure(109)
    hold off
    plot(trust1(:,3), (trust1(:,5)-0.078)*Km./(trust1(:,2)*2*pi).^3)
    Kdrag = mean((trust1(:,5)-0.078)*Km./(trust1(:,2)*2*pi).^3)
    hold on
    plot([0,80],[Kdrag,Kdrag])
    xlabel('RPS')
    legend('14" drag/\omega^3', '14" average drag/\omega^3', ...
           'location','north');
    grid on
    ylabel('Drag konstant [Nm \omega^{-3}]')
    title('3508-700KV MultiStar - 14x5.5" prop');
%% RPS --- drag constant 18"
    figure(110)
    hold off
    % 18"
    plot(trust2(:,3), (trust2(:,5)-0.19)*Km./(trust2(:,2)*2*pi).^3.7, '--')
    hold on
    Kdrag2 = mean((trust2(:,5)-0.19)*Km./(trust2(:,2)*2*pi).^3.7)
    plot([0,80],[Kdrag2,Kdrag2],'--')
    xlabel('RPS')
    legend('18" drag/\omega^3', '18" average drag/\omega^3','location','south');
    grid on
    ylabel('Drag konstant [Nm \omega^{-3}]')
    title('3508-700KV MultiStar - 18x5.5" prop');
%% RPS --- trust
    figure(112)
    hold off
    plot(trust1(:,2), trust1(:,6)*9.82/1000./(trust1(:,3)*2*pi).^2)
    kTrust = mean(trust1(:,6)*9.82/1000./(trust1(:,3)*2*pi).^2)
    hold on
    plot([0,80],[kTrust, kTrust])
    grid on
    title('3508-700KV MultiStar - 14x4.7" prop');
    xlabel('RPs')
    ylabel('Ktrust [N \omega^{-2}]')

