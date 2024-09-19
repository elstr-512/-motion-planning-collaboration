%% imports
addpath("workspace/");
addpath("utils/");
addpath("local_planner/");
addpath("global_planner/");
addpath("utils/env/", "utils/data/map/", "utils/plot/", "utils/animation/");
addpath("local_planner/");
addpath("global_planner/graph_search/");
addpath("global_planner/sample_search/");
addpath(genpath("utils/"), genpath("global_planner/"));

%% Test data
rows = 4;
cols = 30;

% time/distace table
fixedMeanValue = 30.0;
randRange = 9;
distanceTable = fixedMeanValue + randRange * (2 * rand(rows, cols) - 1)';

% mean curvature table
fixedMeanValue = 10.0;
randRange = 3;
curvatureTable = fixedMeanValue + randRange * (2 * rand(rows, cols) - 1)';

% some outliers
curvatureTable (30, 1) = 90.86280;
curvatureTable (30, 2) = 130.86280;
curvatureTable (30, 3) = 49.86280;
curvatureTable (30, 4) = 59.86280;

%% Load data
function [curvatures, lengths, distances] = load_data(file)
    all_data = load(file);
    curvatures = vertcat(all_data.all_curvatures{:});
    curvatures = curvatures(curvatures > 0);
    curvatures = curvatures(curvatures < 6);
    lengths = vertcat(all_data.all_lengths(:));
    distances = vertcat(all_data.all_distances{:});
end

[curvatures_a_star, lengths_a_star, distances_a_star] = load_data("out/data_a_star.mat");
[curvatures_dijkstra, lengths_dijkstra, distances_dijkstra] = load_data("out/data_dijkstra.mat");
[curvatures_voronoi_plan, lengths_voronoi_plan, distances_voronoi_plan] = load_data("out/data_voronoi_plan.mat");

%% Combine data
function [data, idx] = combine_data(all_data)
    data = vertcat(all_data{:});
    lengths = cellfun(@length, all_data);
    idx = [];
    for i = 1:length(lengths)
       idx = [idx; repmat(i, lengths(i), 1)];
    end
end

[curvatures, curvatures_idx] = combine_data({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan});
[lengths, lengths_idx] = combine_data({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan});
[distances, distances_idx] = combine_data({distances_a_star; distances_dijkstra; distances_voronoi_plan});

%% Plotting visuals
algorithms = {'PID-A*', 'PID-Dijkstra', 'PID-Voronoi'};
colororders = [
    0.2627, 0.2235, 0.8863; % #4339e2
    0.0627, 1.0000, 0.2627; % #10ff43
    0.8980, 0.2431, 0.2667; % #e53e44
    0.6274, 0.5098, 0.0352; % #a08209
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
    end
    hold off;
    
    % Add labels and legend
    xticks(1:length(data));
    xticklabels(algorithms);
    ylabel(y_label);
    title(name);
    % legend(algorithms, 'Location', 'eastoutside');
end

make_boxplot({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan}, ...
    colororders, algorithms, "Curvature", "Algorithm Performance - Curvature Comparison");

make_boxplot({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan}, ...
    colororders, algorithms, "Length", "Algorithm Performance - Path Length Comparison");

make_boxplot({distances_a_star; distances_dijkstra; distances_voronoi_plan}, ...
    colororders, algorithms, "Distance", "Algorithm Performance - Distance to Obstacles Comparison");

%% Bar graph curvature
groupedData = curvatureTable(:); % Flatten data into one column
algorithmIndices = repmat(1:rows, cols, 1); % Assign each row to a group (algorithm)
algorithmIndices = algorithmIndices(:); % Flatten to match groupedData

% Create box chart
figure; hold on; colororder(colororders);
% Customize colors for each algorithm
for i = 1:rows
    boxchart(algorithmIndices(algorithmIndices == i), groupedData(algorithmIndices == i));
end
hold off;

% Add labels and legend
xticks(1:rows);
xticklabels(algorithms);
ylabel('2nd Dervivative');
title('Algorithm Performance - Curvature Comparison');
legend(algorithms, 'Location', 'eastoutside');
