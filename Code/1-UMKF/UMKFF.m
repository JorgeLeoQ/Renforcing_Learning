clc
clear
close all
addpath('./dataset')
load('Train_Data.mat')
attractor = false;
addpath('./ekfukf')
% Change names
if attractor == false
    PosNoiseX = PosNoiseX1;
    PosNoiseY = PosNoiseY1;
else
%     TrajectAttrX1 = TrajectObsX1;
%     TrajectAttrY1 = TrajectObsY1;
    TrajectAttrX1 = TrajectAttrX1;
    TrajectAttrY1 = TrajectAttrY1;
    out_PosX = TrajectAttrX1(:, 1);
    out_PosY =  TrajectAttrY1(:,1);
    TrajectAttrX1(:,1) = awgn(out_PosX,10);
    TrajectAttrY1(:,1) = awgn(out_PosY,10);
%     figure;
%     scatter(TrajectAttrX1,TrajectAttrY1,'r','filled')
    xlim([-14 17])
    ylim([-16 0])
    grid on
    PosNoiseX = TrajectAttrX1(:,1);
    PosNoiseY = TrajectAttrY1(:,1);
end
[max_number_points, num_trajectories] = size(PosNoiseX);

% Remove the zeroes after removing zeroes you will have trajectries which
% have different size so you will use the cells to store the data of each
% trajectory as well as store the size of each trajectory to later use
dataC = [];
TrajectorySize =[];
datacell = cell(1,num_trajectories);
for j = 1:num_trajectories
    currentTrajectory = [PosNoiseX(:,j) PosNoiseY(:,j)]';                   % curren trajectory
    currentTrajectory(currentTrajectory==0) = [];                           % remove all zeroes
    sizeTrajectory = size(currentTrajectory,2);                             % size of trajectory after removing zeroes
    if mod(sizeTrajectory,2) == 0                                           % if zero removes from trajectory
        currentTrajectory = reshape(currentTrajectory,2,size(currentTrajectory,2)/2);   % reshape trajectory after removing zeros
    end
    dataC = [dataC; currentTrajectory'];                                     % combine data for all trajectories
    TrajectorySize = [TrajectorySize;  size(currentTrajectory,2)];           % size of each trajecyory
    datacell{j} = currentTrajectory';                                        % data relevant to each trajectory
end

save('datacell.mat', 'datacell');

% plot all trajectories
figure(1); % follower trajectories (training)
hold on
scatter(dataC(:,1),dataC(:,2), 'b');
grid on
box on
title('Testing dataset trajectories');
legend('Followers');
xlabel('x');
ylabel('y');
hold off


%% UMKF  (Null Force Filter)

% Transition matrix for the continous-time system.
A = [1  0  0  0;
    0  1  0  0;
    0  0  1  0;
    0  0  0  1];

% Process noise variance
sig_a = 0.2;
Q = eye(4) * sig_a^2;

% Measurement model.
H = [1 0 0 0;
    0 1 0 0];

% Variance in the measurements.
r1 = 1e-10;
R = diag([r1 r1]);
% Final state dimension is 4: position x, position y, velocity x, velocity y
state_dim = 4;
% Space for the estimates
MM = zeros(size(dataC,1), state_dim); % mean matrix
Abnormalinn = zeros(size(dataC,1), 1);
stateNoise = zeros(size(dataC,1), state_dim);
index = 1; % index the rows of MM matrix (state matrix)
Traj = zeros(size(dataC,1), state_dim); % contains observation on x and y and the computed innovation

for i = 1:num_trajectories
    % select one trajectory
    Y = datacell{i}';                              % Y is the matrix with the observations of position x and y of followers
    % Filtering steps
    
    for j = 1:size(Y,2)
        % Initial guesses for the state mean and covariance.
        if j ==1
            m = [Y(1,1) Y(2,1) 0 0]';
            P = diag([0.1 0.1 0.1 0.1]);
        end
        
        % Prediction
        [m,P] = kf_predict(m,P,A,Q);
        % Update
        [m,P,K,inn,IM,~,~,Abnormalinn1] = kf_update(m,P,Y(:,j),H,R);
        MM(index,1:2) = m(1:2,1)'; % position on x and y
        Traj(index,1:2) = Y(:,j)';
        MM(index,3:4) = inn'; % we can see the innovation as the velocity with the assumption of static object
        Traj(index,3:4) = inn';
        Abnormalinn(index,1) = Abnormalinn1(1,1)';
        stateNoise(index,:) = mvnrnd(m(:,1),P(:,:,1));    % add noise in velocities
        index = index + 1;
        
    end
    
end

% random select a trajectory
a = 1;
b = num_trajectories;
rnd = randi([a b],1,1);   % for avoiding decimal number

% select data corresponding to each trajectory
state_rndCell = cell(1,num_trajectories);
StartPoint = 0;
AbnormalinnCell = cell(1,num_trajectories);
stateNoiseCell = cell(1,num_trajectories);
TrajCell = cell(1,num_trajectories);
for j = 1:1:num_trajectories
    EndPoint = TrajectorySize(j,1);
    e = EndPoint+StartPoint;
    state_rndCell{j} = MM(StartPoint+1:e,:);
    AbnormalinnCell{j} = Abnormalinn(StartPoint+1:EndPoint+StartPoint,:);
    stateNoiseCell{j} = stateNoise(StartPoint+1:EndPoint+StartPoint,:);
    TrajCell{j}= Traj(StartPoint+1:EndPoint+StartPoint,:);
    StartPoint = e;
end
state_rnd = state_rndCell{rnd};
Abnormalinn =  AbnormalinnCell{rnd};
stateNoise = stateNoiseCell{rnd};
figure; % trajectory, filtered trajectory, velocity
hold on
% trajectory
subplot(1,4,1);
scatter(datacell{rnd}(:,1), datacell{rnd}(:,2), 'b');
ylim ([-20 15]);
% ylim ([10 18]);
xlim([-10 15]);
grid on
box on
title('Observations');
xlabel('x');
ylabel('y');
% % filtered trajectory
subplot(1,4,2);
hold on
scatter(state_rnd(:,1), state_rnd(:,2), 'k');
quiver(state_rnd(:,1),state_rnd(:,2),state_rnd(:,3),state_rnd(:,4), 'r','AutoScale','off');
grid on
box on
hold off
legend('Filtered trajectory','Innovation');
title('KF filtering');
xlabel('x');
ylabel('y');
ylim ([-20 15]);
% ylim ([10 18]);
xlim([-10 15]);
subplot(1,4,3);
quiver(state_rnd(:,1),state_rnd(:,2),stateNoise(:,3),stateNoise(:,4), 'r','AutoScale','off');
hold on
scatter(state_rnd(:,1),state_rnd(:,2),'go')
scatter(datacell{rnd}(:,1), datacell{rnd}(:,2), 'k*');
ylim ([-20 15]);
% ylim ([10 18]);
xlim([-10 15]);
grid on
box on
hold off
legend('Measured velocity','Filtered velocity','Real Trajectory');
title('KF filtering');
xlabel('x');
ylabel('y');

% velocity
subplot(1,4,4);
plot(Abnormalinn(:,1));
title('Innovation');
legend('Innovation on x','Innovation on y');
xlabel('time instant');
grid on
box on
hold off
%% Save information
data.datacell = datacell;
data.MMCell = state_rndCell;
data.TrajCell = TrajCell;
data.stateNoiseCell = stateNoiseCell;
data.TrajectorySize = TrajectorySize;
data.MM = MM;
data.stateNoise = stateNoise;
data.Trajectory = Traj;
data.TrajectoryNum = rnd;
data.CurrentTrajectory = state_rnd;
data.num_trajectories = num_trajectories;
save('dataFollower.mat','data')