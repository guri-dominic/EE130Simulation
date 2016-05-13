function [ measure ] = bCdistMeasure(N, allAgentsEstimationLoc, anchorLoc)
% This function takes 
%       N                           - number of agents
%       allAgentsEstimationLoc      - estimated locations of all agents
%       anchorLoc                   - locations of all anchors
%
% - Uses barycentric coords & convex hull nodes [from chooseConvHull()]  to
% measure location of the sensors.
BP = zeros(N, length(allAgentsEstimationLoc));

%% Compute BaryCentric Coords
for agentsIndex = 1:N
    % locs = getAllLocs(index) % get all current locations
    [neighbors, bCoords] = chooseConvHull(agentsIndex, [allAgentsEstimationLoc anchorLoc]); % choose neighboring sensors
    BP(agentsIndex, neighbors(1)) = bCoords(1);
    BP(agentsIndex, neighbors(2)) = bCoords(2);
    BP(agentsIndex, neighbors(3)) = bCoords(3);
end

%% Get B & P Matrices
P = BP(:,(1:N));
B = BP(:,(N+1:end));

%% Get Coords
bCoordsColumns = transpose(BP);
size(allAgentsEstimationLoc)
size(bCoordsColumns)
measure = [allAgentsEstimationLoc anchorLoc] * bCoordsColumns;


% X = 2*rand(N,2);
% xlog = X;
% counter = 0;
% U = transpose(anchorLoc);
% E = ones(size(X));
% 
% while max(max(E)) > 1e-3
%     X = P*X+B*U
%     E = abs(transpose(agentsLoc(a, index)) - (X))
%     counter = counter + 1;
%     
%     % abs(X - transpose(agentsLoc(a, index)))
%     % xlog(:,:,counter+1) = X;
% end
end

