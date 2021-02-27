%%
%script che serve per stampare le traiettorie da mettere nel report
clc;
close all;
clear all;

load('dataFollower.mat');

figure(1)
hold on
grid on 
title('Training dataset trajectories')
scatter(data.Trajectory(:,1), data.Trajectory(:,2), 'b');

load('dataAttractor.mat');
c = scatter(data.Trajectory(:,1), data.Trajectory(:,2));
c.LineWidth = 0.6;
c.MarkerEdgeColor = 'r';
c.MarkerFaceColor = [1 0 0];

xlabel('x');
ylabel('y');
hold off

