function [nodesMean, nodesMean_Pos, nodesMean_Vel] = GetMean(datanodes)
nodesMean = [];
nodesMean_Pos = [];
nodesMean_Vel = [];
for i = 1:size(datanodes,2)
    %   Calculation of mean values
    nodesMean = [nodesMean; mean(datanodes{1,i})];
    nodesMean_Pos = [nodesMean_Pos; mean(datanodes{1,i}(:,1:2))];
    nodesMean_Vel = [nodesMean_Vel; mean(datanodes{1,i}(:,3:4))];
end

end
