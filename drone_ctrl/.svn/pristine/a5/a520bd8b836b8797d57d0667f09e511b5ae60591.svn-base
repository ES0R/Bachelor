% mission plan
% for drone control simulation
close all
clear
%
%% build simulation
% go to height 1m for trust estimation
hexacopter;
%% go to height 1m for trust estimation
% heightInRef.time=[0,1,1,3,3,6,6,8,20]';
% heightInRef.signals.values=[0,0,1,1,5,5,3,3,3]';
heightInRef.time=[0,0.01]';
heightInRef.signals.values=[2,2]';
sim('hexacopter_sim',7);