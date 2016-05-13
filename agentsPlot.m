function agentsPlot(a, index)
    %   Plots the agents at a given time(index)
    %
    %   a         - array of agents
    %   index     - time step

    C = agentsLoc(a, index);

    % Labels
    aV = [1:10]'; aVstr = num2str(aV); aVstrCell = cellstr(aVstr);
    dx = 0.02; dy = -0.2;
    text(C(1,:)+dx, C(2,:)+dy, aVstrCell, 'LineWidth', 3,'FontSize', 18);
    scatter(C(1,:), C(2,:), 140,'filled','or')
    set(gca,'FontSize',16);
end

