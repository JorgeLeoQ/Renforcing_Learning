function [transitionMat, DiscreteTrajectories] = GetTransitionMatrix(N,nodesInTime,trajectorySize,TotalTrajectories)

% Initialize transition matrix
transitionMat = zeros(N,N);

StartPoint = 0;
DiscreteTrajectories = cell(1,TotalTrajectories);
for j = 1:TotalTrajectories
     EndPoint = trajectorySize(j,1);
    e = EndPoint+StartPoint;
    DiscreteTrajectories{j} = nodesInTime(StartPoint+1:e,:);
    StartPoint = e;
end
%% transition Matrix
for t = 1:1:TotalTrajectories
    NodesTime = DiscreteTrajectories{t};
    for k = 1:trajectorySize(t,1)-1
        transitionMat(NodesTime(k,1),NodesTime(k+1,1)) =...
            transitionMat(NodesTime(k,1),NodesTime(k+1,1)) + 1;
    end
    NodesTime = [];
end
% normalization of transiiton matrix
transitionMat = transitionMat./repmat(sum(transitionMat,2) + (sum(transitionMat,2)==0),1,N);

end
