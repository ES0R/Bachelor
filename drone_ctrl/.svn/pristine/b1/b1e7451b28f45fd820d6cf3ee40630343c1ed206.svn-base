%% - plot static trust, current and RPM
% trust using a standard scale 25kg max, 1g resolution
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
trust003 = load('3508-700_13x4.5.txt'); % 11 V 13x4.5
trust003a = load('3508-700_13x4.5_02.txt'); % 15V
trust003b = load('3508-700_13x4.5_02.txt'); % 11.1V
trust004 = load('3508-700_8x3.8.txt'); % 15V  8"x3.6
trust005 = load('3508-700_9x4.5.txt'); % 15V  9"x4.5
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
trust4 = trust005; %9"
trust5 = trust004; %8"
%% trust -- power
vf1 = trust1(:,4);
vf2 = trust2(:,4);
vf3 = trust3(:,4);
vf4 = trust4(:,4);
vf5 = trust5(:,4);
h=figure(100)
    hold off
    plot(trust5(:,5).*vf5, trust5(:,6)*9.82/1000,'-r','linewidth',1);
    hold on
    plot(trust4(:,5).*vf4, trust4(:,6)*9.82/1000,'-gx','linewidth',1);
    plot(trust3(:,5).*vf3, trust3(:,6)*9.82/1000,'-c','linewidth',1);
    plot(trust1(:,5).*vf1, trust1(:,6)*9.82/1000,'-b','linewidth',1);
    plot(trust2(:,5).*vf2, trust2(:,6)*9.82/1000,'-mo','linewidth',1);
    grid on
    legend('trust (N) 8"','trust (N) 9"','trust (N) 13"','trust (N) 14"','trust (N) 18"', ...
        'location','south east');
    xlabel('Motor input power (W)')
    ylabel('Trust (Newton)')
    title('Power efficiency - 3508-700KV MultiStar');
saveas(h,'trust-per-power-3508-700.png')
%% power - RPM
vf1 = trust1(:,4);
vf2 = trust2(:,4);
vf3 = trust3(:,4);
vf4 = trust4(:,4);
vf5 = trust5(:,4);
h=figure(101)
    hold off
    plot(trust5(:,5).*vf5, trust5(:,2)*60,'-r','linewidth',1);
    hold on
    plot(trust4(:,5).*vf4, trust4(:,2)*60,'-gx','linewidth',1);
    plot(trust3(:,5).*vf3, trust3(:,2)*60,'-c','linewidth',1);
    plot(trust1(:,5).*vf1, trust1(:,2)*60,'-b','linewidth',1);
    plot(trust2(:,5).*vf2, trust2(:,2)*60,'-mo','linewidth',1);
    grid on
    legend('8" prop','9" prop','13" prop','14" prop','18" prop',...
        'location','north east');
    xlabel('Motor input power (W)')
    ylabel('RPM')
    title('RPM - 3508-700KV MultiStar');
saveas(h,'rpm-per-power-3508-700.png')
%% ESC -- trust
h = figure(102)
hold off
plot(trust5(:,1)/1000 + 1, trust5(:,6)*0.0098./trust5(:,4)*11,'-r','linewidth',1);
hold on
plot(trust4(:,1)/1000 + 1, trust4(:,6)*0.0098./trust4(:,4)*11,'-gx','linewidth',1);
plot(trust3(:,1)/1000 + 1, trust3(:,6)*0.0098./trust3(:,4)*11,'-c','linewidth',1);
plot(trust1(:,1)/1000 + 1, trust1(:,6)*0.0098./trust1(:,4)*11,'-b','linewidth',1);
plot(trust2(:,1)/1000 + 1, trust2(:,6)*0.0098./trust2(:,4)*11,'-mo','linewidth',1);
grid on
legend('8"x3.6 prop','9"x4.5 prop','13"x4.5 prop', '14"x5.5 prop', ...
    '18"x5.5 prop','location','north west');
xlabel('ESC PW (1-2ms)')
ylabel('Trust (Newton) (scaled to 11V)')
title('3508-700KV MultiStar - static transfer');
saveas(h, 'trust-TF-3508-700.png')
%% transfer function gain
% 8"
for i = 1:size(trust5,1)-1
    dN = trust5(i+1,6) - trust5(i,6)
    dW5(i) = (trust5(i+1,1) - trust5(i,1))/1000
    dNw5(i) = dN/dW5(i);
end
% 9"
for i = 1:size(trust4,1)-1
    dN = trust4(i+1,6) - trust4(i,6)
    dW4(i) = (trust4(i+1,1) - trust4(i,1))/1000
    dNw4(i) = dN/dW4(i);
end
% 13"
for i = 1:size(trust3,1)-1
    dN = trust3(i+1,6) - trust3(i,6)
    dW3(i) = (trust3(i+1,1) - trust3(i,1))/1000
    dNw3(i) = dN/dW3(i);
end
%14"
for i = 1:size(trust1,1)-1
    dN = trust1(i+1,6) - trust1(i,6)
    dW1(i) = (trust1(i+1,1) - trust1(i,1))/1000
    dNw1(i) = dN/dW1(i);
end
%18"
for i = 1:size(trust2,1)-1
    dN = trust2(i+1,6) - trust2(i,6)
    dW2(i) = (trust2(i+1,1) - trust2(i,1))/1000
    dNw2(i) = dN/dW2(i);
end
%% Transfer gain
k0v1 = dNw1'.*0.0098./trust1(2:end,4).*11;
k0v2 = dNw2'.*0.0098./trust2(2:end,4).*11;
k0v3 = dNw3'.*0.0098./trust3(2:end,4).*11;
k0v4 = dNw4'.*0.0098./trust4(2:end,4).*11;
k0v5 = dNw5'.*0.0098./trust5(2:end,4).*11;
%
h = figure(206)
hold off
p(1) = plot(trust5(2:end,1)/1000+1, k0v5,'-r','linewidth',1, 'DisplayName', '8" prop');
hold on
p(2) = plot(trust4(2:end,1)/1000+1, k0v4,'-gx','linewidth',1, 'DisplayName', '9" prop');
p(3) = plot(trust3(2:end,1)/1000+1, k0v3,'-c','linewidth',1, 'DisplayName', '13" prop');
p(4) = plot(trust1(2:end,1)/1000+1, k0v1,'-b','linewidth',1,'DisplayName', '14" prop');
p(5) = plot(trust2(2:end,1)/1000+1, k0v2,'-mo','linewidth',1,'DisplayName', '18" prop');
grid on
xlabel('ESC pulse (ms)')
ylabel('(K_0) Newton per ms (scaled to 11V)')
%legend('13" prop','14" prop','18" prop','location','south')
legend(p,'autoupdate','off','location','north west')
title('Static transfer gain')
saveas(h,'transfer_gain_N-per-ms.png')
