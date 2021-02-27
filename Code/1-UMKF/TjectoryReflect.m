clc
clear all;
close all;

%%
% load('datacell.mat');
% 
% numTraject = size(datacell, 2);
% 
% figure(1);
% hold on;
% for i = 1: size(datacell, 2)
%    scatter(datacell{1,i}(:,1), datacell{1,i}(:,2), 'b');
% end
% grid on
% box on
% xlabel('x');
% ylabel('y');
% hold off
% 
% figure(2);
% hold on;
% for i = 1: size(datacell, 2)
%    scatter(-datacell{1,i}(:,1), datacell{1,i}(:,2), 'b');
% end
% grid on
% box on
% xlabel('x');
% ylabel('y');
% hold off


load('dataFollowerTest');
figure(3);
hold on
scatter(data.Trajectory(:,1), data.Trajectory(:,2), 'b');
scatter(-data.Trajectory(:,1), data.Trajectory(:,2), 'b');
grid on
box on
xlabel('x');
ylabel('y');
hold off


