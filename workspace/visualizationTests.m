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

%% Plotting visuals
algorithms = {'DWA', 'PID-A*', 'PID-Dijkstra', 'PID-Voronoi'};
colororders = [
    0.2627, 0.2235, 0.8863; % #4339e2
    0.0627, 1.0000, 0.2627; % #10ff43
    0.8980, 0.2431, 0.2667; % #e53e44
    0.6274, 0.5098, 0.0352; % #a08209
];

%% Box chart distance
% https://se.mathworks.com/help/matlab/ref/boxchart.html
% usecases: time/distace, collition avoidance
% Prepare data for box chart
groupedData = distanceTable(:); % Flatten data into one column
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
ylabel('Distance');
title('Algorithm Performance - Distance Comparison');
legend(algorithms, 'Location', 'eastoutside');

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
