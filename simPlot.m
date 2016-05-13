function simPlot(a, index)
    N = length(a);
    
    figure
    rectangle('Position',[-5 -5 10 10],'LineWidth',3)
    axis([-6 8 -6 6])
    hold on
        for i = 1:N
            plot(a(i).xn(1,:), a(i).xn(2,:), 'LineWidth', 1)
        end
        agentsPlot(a, index)
    hold off
    legend('1','2','3','4','5','6','7','8','9','10')
    grid on
end

