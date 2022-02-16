% 2 order with no overshoot
close all
clear
%%
ti = 100;
td = 1;
%% design height velocity controller
G = tf([1],[1 1 0])
% P-controller
gm = 75;
al = 1;
Ni = 0;
[w, kp] = findp(G, gm)
% debug and save result
showResult(1,0,'aaa.txt', 'heightVel', G, ...
           w, kp, ti, Ni, td, al, gm, ...
           'P-controller');
%% PI controller
gm = 70;
al = 1;
Ni = 5;
[w, kp,ti, td] = findpi(G, gm, Ni)
% debug and save result
showResult(1,0,'aaa.txt', 'heightVel', G, ...
           w, kp, ti, Ni, td, al, gm, ...
           'PI-controller');
%% PLead controller
gm = 65;
al = 0.1;
Ni = 0;
[w, kp,ti, td] = findpd(G, gm, al)
% debug and save result
showResult(1,0,'aaa.txt', 'heightVel', G, ...
           w, kp, ti, Ni, td, al, gm, ...
           'P-Lead');
%% PI-Lead controller
gm = 70;
al = 0.05;
Ni = 3;
[w, kp,ti, td] = findpid(G, gm, Ni, al)
% debug and save result
showResult(1,0,'aaa.txt', 'heightVel', G, ...
           w, kp, ti, Ni, td, al, gm, ...
           'PI-Lead');
       
Cd = tf([td 1],[al*td 1]);
Ci = tf([ti 1],[ti 0])
Gol = kp*Ci*Cd*G;
%%
figure(1000)
hold off
nyquist(Gol)
axis([-6,0,-3,3])
grid on
%%
Gcl = minreal(kp*Ci*G/(1+kp*Ci*Cd*G));
[Ms,Ps,Wo] = bode(G);
[Mo,Po] = bode(Gol, Wo);
[Mc,Pc] = bode(Gcl,Wo);
figure(100)
hold off
subplot(2,2,1)
hold off
% bode magnitude
semilogx(Wo, 20*log10(squeeze(Ms)))
hold on
semilogx(Wo, 20*log10(squeeze(Mo)))
semilogx(Wo, 20*log10(squeeze(Mc)))
grid on
ylabel('Magnitude');
title('aaa title aaa');
% phase
subplot(2,2,3)
hold off
semilogx(Wo, squeeze(Ps))
hold on
semilogx(Wo, squeeze(Po))
semilogx(Wo, squeeze(Pc))
grid on
ylabel('Phase (deg)');
xlabel('Frequency (rad/s)')
subplot(2,2,2)
hold off
step(Gcl)
grid on
subplot(2,2,4)
hold off
nyquist(Gol)
axis([-6,0,-3,3])
grid on


%%
hold on
margin(Gol);
set(gca,'FontSize',8);
bode(Gcl);
grid on
%xlabel({'\fontsize{10}Frequency (rad/s) ', tle});
legend('system','open loop','closed loop');
subplot(1,2,2)
step(Gcl)
grid on
title(tle,'FontSize',8)
