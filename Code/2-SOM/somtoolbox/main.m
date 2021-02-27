clc
clear
close all
%% Load data set (Continuos state space of follower)
load('Data.mat')
load('numberData.mat')                                                      % Number of data for each trajectory
%% TRAINING SOM
% Weights definition for position (beta), velocity (alpha) where alpha>beta
% to favor follower's velocity
alpha = 0.85/2;
beta = 0.15/2;
[prototype, containerID, dataCode2, containerData,...
    colorsMats,SomSize] = somclustering(alpha, beta, data);
usedNeurons = 1:SomSize;

% %% Plotting
% colSOM = colorsMats.SOM(usedNeurons,:);
% avNmat = prototype;
% h2 = figure;
% hold on;
% s2 = scatter3(avNmat(:,1), avNmat(:,2),avNmat(:,3),50,'k', 'filled');       %   Draws neuron with z=v_x
% scatter3(data(:,1),data(:,2),data(:,3),...                         %   Data on the plane
%     4,colorsMats.Data, 'filled');
% xlab = xlabel('$x$','interpreter','latex');
% ylab = ylabel('$y$','interpreter','latex');
% zlab = zlabel('$\dot{x}$','interpreter','latex');
% xlab.FontSize = 22;
% ylab.FontSize = 22;
% zlab.FontSize = 22;
% grid minor
% % leg2 = legend(s2,'Prototypes projected onto $\dot{x}$');
% % set(leg2,'Interpreter','latex');
% % leg2.FontSize = 14;
% % leg2.Position = [0.3010    0.8293     0.4947    0.0543];
% ax1 = gca;
% ax1.View = [-38.4000 28.4000];
% h2.Position = [607   495   569   486];
% 
% %   Adjust image axes (Start)
% outerpos = ax1.OuterPosition;
% ti = ax1.TightInset;
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax1.Position = [left bottom ax_width-0.01 ax_height];
% %   Adjust image axes (End)
% set(h2,'Units','Inches');
% posPage = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits',...
%     'Inches','PaperSize',[posPage(3), posPage(4)])
% %   yDot components
% h3 = figure;
% hold on;
% s3 = scatter3(avNmat(:,1), avNmat(:,2),avNmat(:,4),70,'k', 'filled');              %   Draws neuron with z=v_x
% scatter3(data(:,1),data(:,2),data(:,4), 4,...
%     colorsMats.Data, 'filled');
% xlab = xlabel('$x$','interpreter','latex');
% ylab = ylabel('$y$','interpreter','latex');
% zlab = zlabel('$\dot{y}$','interpreter','latex');
% xlab.FontSize = 22;
% ylab.FontSize = 22;
% zlab.FontSize = 22;
% grid minor
% % leg3 = legend(s3,'Prototypes projected onto $\dot{y}$');
% % set(leg3,'Interpreter','latex');
% % leg3.FontSize = 14;
% % leg3.Position = [0.2912    0.75    0.4932    0.0543];
% ax2 = gca;
% ax2.View = [-54.8000   28.4000];
% h3.Position = [1215 495 569  486];
% 
% %   Adjust image axes (Start)
% outerpos = ax2.OuterPosition;
% ti = ax2.TightInset;
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax2.Position = [left bottom ax_width-0.01 ax_height];
% 
% %   Adjust image axes (End)
% set(h3,'Units','Inches');
% posPage = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits',...
%     'Inches','PaperSize',[posPage(3), posPage(4)])