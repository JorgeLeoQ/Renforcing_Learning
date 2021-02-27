function estimationAbn = MJPF(averageDiv,covarianceDiv,...
    averageState, covariance, radiusState,radiusDiv,...
    transitionMat,transMatsTime,trainingData, dataTest)

%% Definition of the employed data
dataTest = dataTest';

%% Definition of the parameters
% Transition matrix for the continous-time system.
numState = 2;
A = [eye(numState),zeros(numState);zeros(numState),zeros(numState)];
%   Measurement model.
H = [eye(numState),zeros(numState)];
%   Control input
B = [eye(numState);eye(numState)];
% Variance in the measurements.
r1 = 1e-1;
R = eye(numState)*r1;
R1 = eye(2*numState)*r1;
%% Initialization Filtering steps.
% Initial guesses for the state mean and covariance.
N = 100;                                                                    % Number of particles
Q = eye(numState*2)*r1;                                                     % Prediction noise equal for each superstate
weightsPar = zeros(1,N);                                                    % Weight of particles (1x100 matrix that contains all zeros)

%% Definition of parameters
statepred = zeros(4,size(dataTest,2),N);                                   % predict state 4D object 1
Ppred = zeros(4,4,size(dataTest,2),N);                                     % predict covariance matrix object 1
stateUpdated = zeros(4,size(dataTest,2),N);
updatedP1 = zeros(4,4,size(dataTest,2),N);
weightscoeff = zeros(N,size(dataTest,2));                                   % weights for each particle(pesi per ogni particle)

db2 = zeros(N,size(dataTest,2));
innovation1 = zeros(N,size(dataTest,2));
activeNodes = zeros(N,size(dataTest,2));
abnormMeas = zeros(1,size(dataTest,2));
abnormdb1 = zeros(1,size(dataTest,2));
abnormdb2 = zeros(1,size(dataTest,2));
t = zeros(N,size(dataTest,2));                                              % time for each particle left for the same Superstate(tempo per ogni particella rimasta per lo stesso Superstato)
nSuperStates = size(transitionMat,2);
firstIt = 1;
h = figure;
h.Position =[844 63 487 898];
%% main loop
%   Counter of trajectory samples
for i = 1:size(dataTest,2)
    %%   FIRST BLOCK (Initialization of states/update KFs' info)
    strfinal = size(dataTest,2);
    display([num2str(i) ' out of ' num2str(strfinal)])
    currMeas = dataTest(:,i);
    particle1 = zeros(2,N);
    
    for n = 1:N                                                             %   loop for each particle
        if i == firstIt
            %   Adding uncertainty noise for initializing the current state
            currStatePos = awgn(currMeas(1:2,:),100);
            currStateVel = awgn(currMeas(3:4,:),100);
            currState = [currStatePos; currStateVel];
            currP1 = eye(4)*(r1);                                         % 4x4 matrix contains all zeros except in diagonal elements value specified
            stateUpdated(:,i,n) = currState;
            updatedP1(:,:,i,n) = currP1;
            
        else
            %   NEXT MEASUREMENT APPEARS: UPDATE
            [stateUpdated(:,i,n),updatedP1(:,:,i,n),~] =...
                kf_update(statepred(:,i-1,n),Ppred(:,:,i-1,n),...
                dataTest(1:2,i),H,R);
            %   Association of updated states to variables
            currP1 = updatedP1(:,:,i,n);
            currState = stateUpdated(:,i,n);                               %   updated state of object
            currStatePos = currState(1:2,1);
            currStateVel = currState(3:4,1);
        end
        
        %%   SECOND BLOCK (Calculation of current superstates)
        
        %   TRANSFORMATION OF STATES INTO SUPERSTATES
        [probDistPos, distancePos] =...
            nodesProb(currStatePos',averageState,radiusState);              %   Find probability of being in each superstate
        probDist = 1./distancePos;
        probDist = probDist/sum(probDist);
        pd = makedist('Multinomial','Probabilities',probDist);             %   Multinomial distribution to pick multiple likely particles
        activeNodes(n,i)  = pd.random(1,1);
        t(n) = 1;
        
        %   Calculate time in the same discrete state for each particle
        if i > firstIt
            if (activeNodes(n,i-1)== activeNodes(n,i))
                t(n) = t(n) + 1;                                            % If same pair add 1
            else
                t(n) = 1;                                                   % Else rinizialize by 1
            end
        else
            t(n) = 1;                                                       %   Time spend in a label is initialized as 1
            weightscoeff(n,i) = 1/N;                                        %   Weight of particles
        end
        
        %% THIRD BLOCK (Abnormal measurements' calculation/particle weighting)
        if i > firstIt
            %   CALCULATION OF ABNORMALITY MEASUREMENTS (db2,db1 and innovation)
            db2(n,i) = bhattacharyyadistance(statepred(:,i-1,n)',...        % measure bhattacharrya distance between p(xk/zk-1) and p(zk/xk) object 1
                dataTest(:,i)',Ppred(:,:,i-1,n), R1);
            
            averageMean = [averageState averageDiv];
            db1(n,i) = bhattacharyyadistance(statepred(:,i-1,n)',...
                averageMean(activeNodes(n,i-1),:),Ppred(:,:,i-1,n),...      %   measure bhattacharrya distance between p(xk/zk-1) and p(xk/sk) object 1
                positivedefinite(covariance{activeNodes(n,i-1)}(:,:)));
            
            innovation1(n,i) = pdist2(statepred(1:2,i-1,n)',...
                currMeas(1:2,1)');
            
            weightsPar(n) = weightscoeff(n,i-1)/(db1(n,i)+db2(n,i)+1e-10);           % weights are 1/ db1 and db2 of both agents
            
            %% FOURTH BLOCK (wait for all particles, particle resampling)
            if n == N
                % Innovation Measurements
                abnormMeas(i) = mean(innovation1(:,i));
                
                % d1 Measurements
                abnormdb1(i) = mean(db1(:,i));
                % d2 Measurements
                
                abnormdb2(i) = mean(db2(:,i));
                
                weightsPar = weightsPar/sum(weightsPar);                   %   Normalize weights in such a way that they all sum 1
                weightscoeff(:,i)= weightsPar';                            %   Assign weights
                
                pd = makedist('Multinomial','Probabilities',weightsPar);   %   Multinomial distribution to pick multiple likely particles
                wRes = pd.random(1,N);                                     %   Take N random numbers from the discrete distribution
                
                for ij = 1:N
                    % REPLACEMENT OF CORRECTED DATA DEPENDING ON
                    % SURVIVING NEURONS
                    temp1 = stateUpdated();
                    temp2 = updatedP1();
                    temp3 = activeNodes();
                    temp4 = t();
                    stateUpdated(:,i,ij) = temp1(:,i,wRes(ij));
                    updatedP1(:,:,i,ij) = temp2(:,:,i,wRes(ij));
                    activeNodes(ij,i) = temp3(wRes(ij),i);
                    t(ij) = temp4(wRes(ij));
                    currState = stateUpdated(:,i,ij);
                    currP1 = updatedP1(:,:,i,ij);
                    weightscoeff(ij,i) = 1/N;                               % Weight of particles
                    
                    %% FIFTH BLOCK (prediction of next discrete and continous levels)
                    %   PREDICTION DISCRETE PART (LABELS)
                    transitionCurr = zeros(1,nSuperStates);
                    transitionCurr(1,1:nSuperStates) = transitionMat(activeNodes(ij,i),:);% Pick row of couple superstate(i-1)
                    if(t(n) <= size(transMatsTime,2))                               %   If time staying in a single region is normal
                        matTr = transitionCurr(1,1:nSuperStates);
                        matTime = transMatsTime{1,t(n)}(activeNodes(n,i),:)+1e-10;  %
                        transitionCurr(1,1:nSuperStates) = (matTr).*matTime;        %   multiply by time interval probability
                    else                                                            %   If time staying in a single region is abnormal
                        transitionCurr(1,1:nSuperStates) = transitionCurr(1,1:nSuperStates);
                    end
                    sum3 = sum(transitionCurr);
                    if(sum3~=0)
                        transitionCurr = transitionCurr/sum3;                       %   Normalize probabilities
                    end
                    velPredictProbs = makedist('Multinomial','Probabilities',...    %   probability of label of two neurons
                        transitionCurr);
                    velPredict(ij,i) = velPredictProbs.random(1,1);                 % Predicted label
                    
                    %** KALMAN FILTER:
                    % PREDICTION CONTINUOS PART
                    U1 = averageDiv(velPredict(ij,i),1:2)';
                    [statepred(:,i,ij),Ppred(:,:,i,ij)] =...                        %   predicition object1
                        kf_predict(currState,currP1, A, Q, B, U1);
                    particle1(:,ij) = statepred(1:2,i,ij);
                    
                end
            end
        end
        
        %% SIXTH BLOCK (prediction of next discrete and continous levels)
        if i == firstIt
            %   PREDICTION DISCRETE PART (LABELS)
            transitionCurr = zeros(1,nSuperStates);
            transitionCurr(1,1:nSuperStates) =...                           %   Pick row of couple superstate(i-1)
                transitionMat(activeNodes(n,i),:);
            if(t(n) <= size(transMatsTime,2))                               %   If time staying in a single region is normal
                matTr = transitionCurr(1,1:nSuperStates);
                matTime = transMatsTime{1,t(n)}(activeNodes(n,i),:)+1e-10;
                transitionCurr(1,1:nSuperStates) = (matTr).*matTime;        %   multiply by time interval probability
            else                                                            %   If time staying in a single region is abnormal
                transitionCurr(1,1:nSuperStates) = transitionCurr(1,1:nSuperStates);
            end
            sum3 = sum(transitionCurr);
            if(sum3~=0)
                transitionCurr = transitionCurr/sum3;                       %   Normalize probabilities
            end
            velPredictProbs = makedist('Multinomial','Probabilities',...    %   probability of label of two neurons
                transitionCurr);
            velPredict(n,i) = velPredictProbs.random(1,1);                  % Predicted label
            
            %** KALMAN FILTER: PREDICTION
            U1 = averageDiv(velPredict(n,i),1:2)';                          %   From GNG1
            [statepred(:,i,n),Ppred(:,:,i,n)] = ...
                kf_predict(currState,currP1, A, Q, B, U1);                 %predicition object1
            particle1(:,n) = statepred(1:2,i,n);
        end
    end
    
    %% SEVENTH BLOCK (Ploting trajectories and abnormality measurements)
    subplot(4,1,1);
    cla
    hold on
    scatter(dataTest(1,1:i),dataTest(2,1:i),'o')
    scatter(trainingData(1:end,1),trainingData(1:end,2),'+r')
    scatter(particle1(1,:),particle1(2,:),'.k');
    title('trajectory')
    grid on
    subplot(4,1,2);
    cla
    plot(abnormdb1(2:i),'-r')
    title('db1')
    grid on
    subplot(4,1,3);
    cla
    plot(abnormMeas(2:i),'-g')
    title('Innovation')
%     threshold = mean(abnormMeas) + 2*std(abnormMeas);
%     line([i 1], [threshold threshold]);
    yline(1.581171e+00);
    grid on
    subplot(4,1,4);
    cla
    plot(abnormdb2(2:i),'-b')
    title('db2')
    grid on
    pause(0.1)
    
end
% fprintf("%d", threshold);
estimationAbn.TrajPred = statepred;
estimationAbn.TrajUpdate = stateUpdated;
estimationAbn.Dataset = dataTest;
estimationAbn.AbnSignaldb2 = abnormdb2;
estimationAbn.Innovation = abnormMeas;
estimationAbn.AbnSignaldb1 = abnormdb1;
end
