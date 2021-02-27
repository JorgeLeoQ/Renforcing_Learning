%% Properties of clusters (Vocabulary)
clc
clear all
close all

% load the output of SOM (clustered data)
load('VocabularySOMA.mat')
DataLength = size(net.data,1) ;
nodesInTime = net.Discrete_data;
datanodes = net.datanodes ;
N = net.N;
trajectorySize = net.UMKF.TrajectorySize;
TotalTrajectories = net.UMKF.num_trajectories;

% %% addition
% usedNeurons = find(~cellfun('isempty',datanodes));  % find the number of used neurons
% datanodes = datanodes(~cellfun('isempty',datanodes));  % remove all the datanodes corresponding to dead nodes
% 
% w = net.w(usedNeurons,:);   % remove all the prototypes corresponding to dead nodes (you should also remove those nodes which conatin only single datasample)
% nodesMean = w;
%%
[nodesMean,nodesMean_Pos ,nodesMean_Vel ] = GetMean(datanodes);
[nodesCov ,nodesCov_Pos ,nodesCov_Vel ] = GetCovariance(datanodes);
[nodesRadAccept, nodesRadAccept_Pos,nodesRadAccept_Vel ]  = GetRadius(datanodes);
[transitionMat, DiscreteTrajectories] = GetTransitionMatrix(N,nodesInTime,trajectorySize,TotalTrajectories);
timeMats = GetTemporalTimeMat(N,nodesInTime);

% store vocabulary(properties of clusters)
net.nodesMean = nodesMean;
net.nodesMean_Pos = nodesMean_Pos;
net.nodesMean_Vel = nodesMean_Vel;

net.nodesCov = nodesCov;
net.nodesCov_Pos = nodesCov_Pos;
net.nodesCov_Vel = nodesCov_Vel;

net.nodesRadAccept = nodesRadAccept;
net.nodesRadAccept_Pos = nodesRadAccept_Pos;
net.nodesRadAccept_Vel = nodesRadAccept_Vel;

net.transitionMat = transitionMat;
net.timeMats = timeMats;
net.DiscreteTrajectories = DiscreteTrajectories;
save('VocabularySOMA.mat','net')

% plot
mycolors = colorcube;
radius = nodesRadAccept;

h = figure;
hold on
scatter(net.data(:,1),net.data(:,2),60,mycolors(nodesInTime,:),'.','LineWidth',1)    % colored input data
scatter(nodesMean(:,1),nodesMean(:,2),250,'+','k','linewidth',2)                     % for the '+' at mean position of nodes
quiver(nodesMean(:,1),nodesMean(:,2),nodesMean(:,3),nodesMean(:,4),'LineWidth',1.8,'Color','r','AutoScale','on', 'AutoScaleFactor', 0.4)
hold on
% for  numbering of nodes
a = [1:N]'; b = num2str(a); c = cellstr(b);
text(net.w(:,1), net.w(:,2), c,'linewidth',4);
hold on
grid on


