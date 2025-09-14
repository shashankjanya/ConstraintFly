function plot_results(T)
    figure;
    hold on;
    
    scatter3(T.weight, T.planformarea, T.score, 35, T.score, 'filled');
    
    xlabel('Weight [N]');
    ylabel('Planform Area [m^2]');
    zlabel('Score');
    title('Weight vs Planform Area vs Score');
    colorbar;
    colormap(jet)
    grid on;
    view(45, 25);
end