function [nodesCov, nodesCov_Pos, nodesCov_Vel] = GetCovariance(datanodes)
for i = 1:size(datanodes,2)
    %   Calculation of covariance values
    nodesCov{1,i} = cov(datanodes{1,i});
    nodesCov_Pos{1,i} = cov(datanodes{1,i}(1,1:2));
    nodesCov_Vel{1,i} = cov(datanodes{1,i}(1,3:4));
end
end