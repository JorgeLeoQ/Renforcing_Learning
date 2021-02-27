%% Fourth  Algorithm: (Abnormality detection)
close all
clear
clc
addpath('./ekfukf')
set(0,'defaultfigurecolor',[1 1 1])
datatTrainigBool = true;
attractor = false;
load('VocabularySOMF.mat')                                                   %   Load learned models
%% Data for testing
if datatTrainigBool == true
    if attractor == false
        trajectoryNum = 12;
        testingData  = net.UMKF.MMCell{trajectoryNum};
    else
        
        testingData  = net.UMKF.MMCell{1,1};
    end
    
else
    if attractor == false
        load('dataFollowerTest.mat')
        trajectoryNum = 8;
        testingData = data.MMCell{trajectoryNum};
    else
        load('dataAttractorTest.mat')
        testingData = data.MMCell{1,1};
    end
    
end

%% Mean and Covariance (training data)

averageState = net.nodesMean_Pos;                                            %   Mean neurons of position data
averageDiv = net.nodesMean_Vel;                                              %   Mean neurons of velocity data

covarianceState = net.nodesCov_Pos;                                          %   Covariance of position data
covarianceDiv = net.nodesCov_Vel;

covariance = net.nodesCov;


radiusState = net.nodesRadAccept_Pos;                                        %   Acceptance neuron radius of position data
radiusDiv = net.nodesRadAccept_Vel;                                          %   Acceptance neuron radius of velocity data

transMatsTime = net.timeMats;                                                %   Transition time of Som of couples (labels)
transitionMat = net.transitionMat;                                           %   Transition time of Som of couples (labels)

trainingData =  net.data;
%% MJPF application
estimationAbn = MJPF(averageDiv, covarianceDiv,...
    averageState, covariance, radiusState,...
    radiusDiv, transitionMat,transMatsTime,trainingData, testingData);

%% threshold


%% Plot Results
% plot learned trajectory i.e., boundary of clusters, number of clusters

t = figure;
t.Position =[544 100 987 898];
hold on
a = [1:net.N]'; b = num2str(a); c = cellstr(b);
text(averageState(:,1)+0.5, averageState(:,2)+0.5, c,'linewidth',4);
quiver(averageState(:,1),averageState(:,2),averageDiv(:,1),averageDiv(:,2),'b','AutoScale','off');
scatter(averageState(:,1),averageState(:,2),'ob')
for a = 1:net.N
    x = averageState(a,1)- radiusState(a);
    y = averageState(a,2) - radiusState(a);
    posPage = [x y 2*radiusState(a) 2*radiusState(a)];
    r = rectangle('Position',posPage,'Curvature',[1 1], 'LineStyle', ':');
end
%scatter(data.Trajectory(:,1), data.Trajectory(:,2), 'p');
title('Figure 2')
for i = 1:size(testingData,1)
    q = quiver(testingData(i,1), testingData(i,2), testingData(i,3), testingData(i,4),'r', 'Autoscale','off');
    q.MaxHeadSize = 5;
    quiver(estimationAbn.TrajPred(1,i,end),estimationAbn.TrajPred(2,i,end),estimationAbn.TrajPred(3,i,end),estimationAbn.TrajPred(4,i,end),'k','AutoScale','off');
    scatter(estimationAbn.TrajPred(1,i,end),estimationAbn.TrajPred(2,i,end),'+k')
    quiver(estimationAbn.TrajUpdate(1,i,end),estimationAbn.TrajUpdate(2,i,end),estimationAbn.TrajUpdate(3,i,end),estimationAbn.TrajUpdate(4,i,end),'g','AutoScale','off');
    scatter(estimationAbn.TrajUpdate(1,i,end),estimationAbn.TrajUpdate(2,i,end),'+g')
    box on
    grid on
    pause(0.5)
end
