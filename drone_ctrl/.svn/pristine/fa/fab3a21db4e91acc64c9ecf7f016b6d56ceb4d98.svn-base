% plot af opti-track data
close all
clear
%%
% log of all rigid body data from optitrack
% 1 Timestamp in seconds
% 2 frameTime (seconds from OptiTrack)
% 3 ID
% 4 valid
% 5 position x
% 6 position y
% 7 position z
% 8 orientation q1 (quertonions)
% 9 orientation q2
% 10 orientation q3
% 11 orientation q4
% 12 position error
dd01 = load('log_opti_rigid.txt');

dd = dd01;
%%
nn = 30000:41000;
k=1;
for i=nn 
    if dd(i,4) == 1053
        k = k+1;
    end
end
ff = zeros(k-1,13);
%%
eu = zeros(k-1,3);
m = 1;
for i=nn
    if dd(i,4) == 1053
      ff(m,:) =  dd(i,:);
      eu(m,:) = quat2eul(dd(i,9:12), 'xyz');
      m = m + 1;
    end
end
%% plot euler angles
%% seems right, but assume x is south, y is east and z is up
% returns [yaw, pitch, roll]
figure(200)
hold off
plot(ff(:,3) - ff(1,3), eu(:,1))
hold on
plot(ff(:,3) - ff(1,3), eu(:,2))
plot(ff(:,3) - ff(1,3), eu(:,3))
legend('x->', 'y->', 'z')
%% plot quaternion
figure(100)
hold off
plot(ff(:,3) - ff(1,3), ff(:,6) - ff(1,6));
hold on
plot(ff(:,3) - ff(1,3), ff(:,7) - ff(1,7));
plot(ff(:,3) - ff(1,3), ff(:,8) - ff(1,8));
plot(ff(:,3) - ff(1,3), ff(:,9));
grid on
plot(ff(:,3) - ff(1,3), ff(:,10));
plot(ff(:,3) - ff(1,3), ff(:,11));
plot(ff(:,3) - ff(1,3), ff(:,12));
legend('x','y','z','q1','q2','q3','q4');
xlabel('sec')
ylabel('vector')
title('Hoop 4 rotation')
%% test
v = ff(1,9)^2 + ff(1,10)^2 + ff(1,11)^2 + ff(1,12)^2
%% to euler
eu = quat2eul(ff(1,9:12))