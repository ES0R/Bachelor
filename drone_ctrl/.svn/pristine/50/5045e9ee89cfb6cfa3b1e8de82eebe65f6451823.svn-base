%% - plot static trust, current and RPM
% trust using a standard scale 25kg max, 1g resolution
close all
clear
%% Trust measurements
% Measurement from esc,motor,propeller test
% file created 2021-11-04 13:13:32.188673
% 1: esc value (motor 1) 0=1ms, 1024 = 2ms
% 2: rps (motor 1) a rotations per second)
% 3: rps (motor 1) b rotations per second)
% 4: Motor voltage (volt)
% 5: total current (amps)
% 6: trust force (gram force)
% 7: CCV (rotation direction)
% 8: Temperature motor (deg C)
% 9: Temperature ESC (deg C)
trust001 = load('../sunny_300kv_22x6.5_22V_01.txt'); % 22V
trust002 = load('../sunny_300kv_24x7.5_22V_02.txt'); % 22V
trust003 = load('../sunny_300kv_18x5.5_22V.txt'); % 22V
%% motor constant
%motor = 'Sunny5210_300kv';
%prop = '22x6.5';
Kv = 300; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
Ra = 0.083; % ohm
%% plot current-trust
trust1 = trust001; %22"
trust2 = trust002; %24"
trust3 = trust003; %18"
%% trust -- power
vf1 = trust1(:,4);
vf2 = trust2(:,4);
vf3 = trust3(:,4);
h=figure(100)
    hold off
    plot(trust1(:,5).*vf1, trust1(:,6)*9.82/1000,'-ro','linewidth',1);
    hold on
    plot(trust2(:,5).*vf2, trust2(:,6)*9.82/1000,'-gx','linewidth',1);
    plot(trust3(:,5).*vf3, trust3(:,6)*9.82/1000,'-bv','linewidth',1);
    plot(trust1(:,5).*vf1, trust1(:,6)*9.82/10./(trust1(:,5).*vf1),':ro','linewidth',1);
    plot(trust2(:,5).*vf2, trust2(:,6)*9.82/10./(trust2(:,5).*vf2),':gx','linewidth',1);
    plot(trust3(:,5).*vf3, trust3(:,6)*9.82/10./(trust3(:,5).*vf3),':bv','linewidth',1);
    grid on
    legend('trust (N) 22"','trust (N) 24"','trust (N) 18"','g/W 22"', 'g/W 24"', 'g/W 18"', ...
        'location','north west');
    xlabel('Motor input power (W)')
    ylabel('Trust (Newton) and g/Watt')
    title('Power efficiency - 5210-300KV Sunny');
saveas(h,'trust-per-power-5210-300.png')
%% power - RPM
h=figure(101)
    hold off
    plot(trust1(:,5).*vf1, trust1(:,2)*60,'-ro','linewidth',1);
    hold on
    plot(trust2(:,5).*vf2, trust2(:,2)*60,'-gx','linewidth',1);
    plot(trust3(:,5).*vf3, trust3(:,2)*60,'-bv','linewidth',1);
    grid on
    legend('22" prop','24" prop','18" prop',...
        'location','east');
    xlabel('Motor input power (W)')
    ylabel('RPM')
    title('RPM - 5210-300KV MultiStar');
saveas(h,'rpm-per-power-5210-300.png')
%% ESC -- trust
h = figure(102)
hold off
plot(trust1(:,1)/1000 + 1, trust1(:,6)*0.0098,'-ro','linewidth',1);
hold on
plot(trust2(:,1)/1000 + 1, trust2(:,6)*0.0098,'-gx','linewidth',1);
plot(trust3(:,1)/1000 + 1, trust3(:,6)*0.0098,'-bv','linewidth',1);
grid on
legend('22"x6.5 prop','24"x7.5 prop','18"x5.5 prop', ...
    '18"x5.5 prop','location','north west');
xlabel('ESC PW (1-2ms)')
ylabel('Trust (Newton)')
title('5210-300KV MultiStar - static transfer');
saveas(h, 'trust-TF-5210-300.png')
%% transfer function gain
% 22"
for i = 1:size(trust1,1)-1
    dN = trust1(i+1,6) - trust1(i,6)
    dW1(i) = (trust1(i+1,1) - trust1(i,1))/1000
    dNw1(i) = dN/dW1(i);
end
% 24"
for i = 1:size(trust2,1)-1
    dN = trust2(i+1,6) - trust2(i,6)
    dW2(i) = (trust2(i+1,1) - trust2(i,1))/1000
    dNw2(i) = dN/dW2(i);
end
% 18"
for i = 1:size(trust3,1)-1
    dN = trust3(i+1,6) - trust3(i,6)
    dW3(i) = (trust3(i+1,1) - trust3(i,1))/1000
    dNw3(i) = dN/dW3(i);
end
%% Transfer gain
k0v1 = dNw1'.*0.0098./trust1(2:end,4).*22;
k0v2 = dNw2'.*0.0098./trust2(2:end,4).*22;
k0v3 = dNw3'.*0.0098./trust3(2:end,4).*22;
%
h = figure(206)
hold off
p(1) = plot(trust1(2:end,1)/1000+1, k0v1,'-ro','linewidth',1, 'DisplayName', '22" prop');
hold on
p(2) = plot(trust2(2:end,1)/1000+1, k0v2,'-gx','linewidth',1, 'DisplayName', '24" prop');
p(3) = plot(trust3(2:end,1)/1000+1, k0v3,'-bv','linewidth',1, 'DisplayName', '18" prop');
grid on
xlabel('ESC pulse (ms)')
ylabel('(K_0) Newton per ms (scaled to 22V)')
%legend('22" prop','24" prop','location','south')
legend(p,'autoupdate','off','location','north west')
title('Static transfer gain')
saveas(h,'5210-300-transfer_gain_N-per-ms.png')
