function [neighbors, bCoords] = chooseConvHull(agentsIndex, agLoc)
% agentsIndex   - index of agent in convex hull
% agLoc         - array of all agents and anchors

search = 1;
cAgent = agLoc(:,agentsIndex);

while search
    q = randi(length(agLoc),[1 3])
%     q = [1 1 7]
%     ~sum(diff([q q(1)]) == 0) || sum(q == agentsIndex)

    % check repeat anchor AND anchor same as current index
    if ~((numel(unique(q(:))) ~= numel(q)) || (sum(q == agentsIndex)>0))
        
        % if not all anchors
        if (~(sum(q >= 11) == 3))
            % check if in convex hull
            A = agLoc(:,q(1)); B = agLoc(:,q(2)); C = agLoc(:,q(3));
            areaT = polyarea([A(1) B(1) C(1)], [A(2) B(2) C(2)]);
            areaA = polyarea([cAgent(1) B(1) C(1)], [cAgent(2) B(2) C(2)]);
            areaB = polyarea([A(1) cAgent(1) C(1)], [A(2) cAgent(2) C(2)]);
            areaC = polyarea([A(1) B(1) cAgent(1)], [A(2) B(2) cAgent(2)]);
            
            % in convex hull
            xy = [A B C]';
            tess = convhulln(xy);
            in = inhull(transpose(cAgent), [xy(:,1) xy(:,2)]);
            if (in)
                search = false;
            else
                % disp('not in convex hull')
            end
        else
            % disp('all anchors')
        end
    else
        % disp('repeat anchor AND anchor same as current index')
    end
end

neighbors = q;
bCoords = [areaA areaB areaC]./areaT;
end

