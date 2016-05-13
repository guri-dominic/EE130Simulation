% http://mathworld.wolfram.com/vanderPolEquation.html
% N=10 sensors/agents located inside a box. 
% M=4 anchor nodes at each corner of the box (location known).
% We assume each sensor knows it’s initial position exactly as an algorithm such
%       as DILOC could be applied for the initial localization.
%
% [LOOP]:
%   1. Move all sensors/agents one step forward according to their motion model
%   2. Compute triangulation / convex hull set for all sensors/agents 
%      (like what we did for DILOC in the second reading)
%   3. For each agent:
%       3a. Measure distances (with noise) between all other sensors/anchors 
%           in its triangulation set
%       3b. Compute position via barycentric coordinate transform using those 
%           distances
%       3c. Use computed position as our measurement step for the various 
%           estimators (EKF, NBF, FPF)
%       3d. Update our estimated position
% [REPEAT]
%
%% Clear & Start Simulation
% clear; close all; clc

%% Anchors
anchorDist = 5;         
anchorLoc = [-5 -5 5 5; ...
            5 -5 -5 5];

%% Particles
N = 10;             % number of particles
M = 4;              % number of anchors
time = (0:0.1:30);      % simulation time

MU = [0.1238 0.7981 0.5518 0.2222 0.3423 0.1389 0.6826 0.7922 0.0250 0.5919];
K = [1 4 3 3 2 1 1 4 3 1];
% Initial Conditions
X_0 = [2.3646 -1.8651 -1.6148 3.1161 -3.7626 2.4792 -1.7582 -2.2639 2.5350 -1.7056];
Y_0 = [1.4502 -2.7035 1.7500 1.3389 -3.8156 -4.1555 -3.0425 -0.6023 0.1007 -2.5684];

a(1,N) = agent(); C = zeros(2,N);
for n=1:N
    mu = MU(n); k = K(n);
    initial_cond = [X_0(n) Y_0(n)];
    a(n) = agent(@(t,y) [y(2); mu*(1 - y(1)^2) * y(2) - k * y(1)], ...
           initial_cond, time);
    
    initialize(a(n)); run(a(n));
    C(:,n) = a(n).x(:,round(length(time)/2));
end

%% Locations Function
getAllLocs = @(index) [agentsLoc(a, index) anchorLoc];
allAgents = M + N;


%% Simulation
% agentslocs = agentsLoc(a, index);
measuredLocs = bCdistMeasure(N, agentslocs, anchorLoc);
% dL = max(max(abs((agentslocs - measuredLocs)./agentslocs)))*100

%% Plot 
% figure 
% hold on
% simPlot(a)
% agentsPlot(a, 1)
% hold off







