%% imports
addpath(genpath("./"));

%% Test data


%% Load data
function [curvatures, lengths, distances] = load_data(file)
    all_data = load(file);
    curvatures = vertcat(all_data.all_curvatures{:});
    curvatures = curvatures(curvatures > 0);
    curvatures = curvatures(curvatures < 6);
    lengths = vertcat(all_data.all_lengths(:));
    distances = vertcat(all_data.all_distances{:});
end

[curvatures_a_star, lengths_a_star, distances_a_star] = load_data("workspace_2/run_planners_space/out/data_a_star.mat");
[curvatures_dijkstra, lengths_dijkstra, distances_dijkstra] = load_data("workspace_2/run_planners_space/out/data_dijkstra.mat");
[curvatures_voronoi_plan, lengths_voronoi_plan, distances_voronoi_plan] = load_data("workspace_2/run_planners_space/out/data_voronoi_plan.mat");
[curvatures_gbfs, lengths_gbfs, distances_gbfs] = load_data("workspace_2/run_planners_space/out/data_gbfs.mat");
[curvatures_theta_star, lengths_theta_star, distances_theta_star] = load_data("workspace_2/run_planners_space/out/data_theta_star.mat");


%% Combine data
function [data, idx] = combine_data(all_data)
    data = vertcat(all_data{:});
    lengths = cellfun(@length, all_data);
    idx = [];
    for i = 1:length(lengths)
       idx = [idx; repmat(i, lengths(i), 1)];
    end
end

[curvatures, curvatures_idx] = combine_data({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan; curvatures_gbfs; curvatures_theta_star});
[lengths, lengths_idx] = combine_data({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan; lengths_gbfs; lengths_theta_star});
[distances, distances_idx] = combine_data({distances_a_star; distances_dijkstra; distances_voronoi_plan; distances_gbfs; distances_theta_star});

%% Plotting visuals
algorithms = {'PID-A*', 'PID-Dijkstra', 'PID-Voronoi', 'gbfs', 'theta_star'};
colororders = [
    0.2627, 0.2235, 0.8863; % #4339e2
    0.0627, 1.0000, 0.2627; % #10ff43
    0.8980, 0.2431, 0.2667; % #e53e44
    0.6274, 0.5098, 0.0352; % #a08209
    0.6274, 0.5098, 0.0352; % #a08209 TODO NEED ANOTHER COLOR
];

%% Box chart length
% https://se.mathworks.com/help/matlab/ref/boxchart.html
% usecases: time/distance, collision avoidance
% Prepare data for box chart

function make_boxplot(data, colororders, algorithms, y_label, name)
    figure; hold on; colororder(colororders);
    % Customize colors for each algorithm
    for i = 1:length(data)
        boxchart(repmat(i, length(data{i}), 1), data{i});

         plot(i, mean(data{1}), 'square', ...
             'MarkerFaceColor', 'k','MarkerEdgeColor', 'k' ...
         );
    end
    hold off;
    
    % Add labels and legend
    xticks(1:length(data));
    xticklabels(algorithms);
    ylabel(y_label);
    title(name);
    % legend(algorithms, 'Location', 'eastoutside');
end

make_boxplot({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan; curvatures_gbfs; curvatures_theta_star}, ...
    colororders, algorithms, "Curvature", "Algorithm Performance - Curvature Comparison");

make_boxplot({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan; lengths_gbfs; lengths_theta_star}, ...
    colororders, algorithms, "Length", "Algorithm Performance - Path Length Comparison");

make_boxplot({distances_a_star; distances_dijkstra; distances_voronoi_plan; distances_gbfs; distances_theta_star}, ...
    colororders, algorithms, "Distance", "Algorithm Performance - Distance to Obstacles Comparison");


