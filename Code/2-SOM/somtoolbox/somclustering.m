function[M,containerID, dataCode, containerData, colorsMats,SomSize] =...
    somclustering(alpha, beta, data, m1, m2,SomSize)
%% SOM takes input training set
[a,~] = size(data);                                                         %   Size a is number of input vectors
%  m1 = 5;
%  m2 = 5;
%  SomSize = m1*m2;
%% SOM algorithm
%initialization of SOM
smI = som_lininit(data, 'msize', [m1 m2] , 'lattice', 'hexa', ...
    'shape', 'sheet');
%   SOM Training
sm = som_batchtrainDistance(smI, data, alpha, beta,...                     
    'radius', [5 .5], 'trainlen', 50, 'neigh', 'gaussian');
M = sm.codebook;                                                           %   M are the prototypes(neurons) of genererated neurons
colorsSOM = som_coloring(sm);                                              %   Colors that group the different prototypes
h = gcf;
som_cplane('hexa', [m1 m2], colorsSOM);                                    %   Draw of similarity between generated prototypes
%h.Position = [2507 189 756 696];
%   divide in 4 components neuron to compute the  distance
M1 = M(:,1);
M2 = M(:,2);
M3 = M(:,3);
M4 = M(:,4);
%% Calculation of membership of data to each generated prototype
containerNumbData = zeros(m1*m2,1);                                         %   Counter of data in neuron
containerID = cell(1,m1*m2);                                                %   Indexes of data in neuron
containerData = cell(1,m1*m2);                                              %   Vector of data in neuron
dataCode = zeros(a,1);                                                      %   Coded data (neuron where it was encoded)
colorsData = [];
%   Distances between data and prototypes
for i = 1: a                                                                %   For all input vectors
    I1 = data(i,1);
    I2 = data(i,2);
    I3 = data(i,3);
    I4 = data(i,4);
    Y = sqrt(beta*((M1 - I1).^2) + beta*((M2 - I2).^2) +...                 %   Weighted distance between all centroids and a particular data point (entire system vector)
        alpha*((M3 - I3).^2) + alpha*((M4 - I4).^2));
    [dist,c] = min(Y);                                                      %   Find minimum neuron to the state in question
    dataCodeDist(i) = dist;                                                 %   Distance to the closes neuron
    dataCode(i) = c;                                                        %   Closest neuron to the data in question
    containerNumbData(c,1) = containerNumbData(c) + 1;                                                  %   Add one to counter of input data in c unit
    containerID{c}(containerNumbData(c),:) = i;                                          %   Indexes of data asociated to each neuron
    containerData{c}(containerNumbData(c),:) = data(i,:);                                %   Add input vector to neuron
    colorsData = [colorsData; colorsSOM(c,:)];
end
colorsMats.Data = colorsData;
colorsMats.SOM = colorsSOM;
%% Calculation of additional functions
drawNumberOfNeurons(containerNumbData,m1, m2);                              %   Number of vector in nodes
unifiedMatrix(sm);                                                          %   Display unified matrix
end