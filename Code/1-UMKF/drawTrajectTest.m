%%
%script che serve per stampare le traiettorie da mettere nel report
clc;
close all;
clear all;

load('dataFollowerTest.mat');

figure(1)
hold on
grid on 
title('Testing dataset trajectories')
c = scatter(data.Trajectory(:,1), data.Trajectory(:,2), 'b');

load('dataAttractorTest.mat');
%c = scatter(data.Trajectory(:,1), data.Trajectory(:,2));
c.LineWidth = 0.6;
c.MarkerEdgeColor = 'g';
c.MarkerFaceColor = [0 1 0];

load('dataAttractor.mat');
%c = scatter(data.Trajectory(:,1), data.Trajectory(:,2));
c.LineWidth = 0.6;
c.MarkerEdgeColor = 'b';
c.MarkerFaceColor = [0 0 1];

xlabel('x');
ylabel('y');
hold off


