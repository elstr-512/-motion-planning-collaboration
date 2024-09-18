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

%% initialize map
clear all;
clc;

% load environment
load("my_gridmap.mat");
map_size = size(grid_map);
G = 1;

% start & goal & pose (pose has to be ajusted for some planners ...)
sx = 4; sy = 4; gx = 46; gy = 36;

start = [sy, sx, pi / 2];
goal = [gy, gx, pi / 2];

%% initialize planner
% global_planner_name = "a_star";
 global_planner_name = "dijkstra";
% global_planner_name = "voronoi_plan";

local_planner_name = "pid_plan";
%local_planner_name = "dwa_plan";

%% Path planning (global)
global_planner = str2func(global_planner_name);
[path, global_flag, cost, expand] = global_planner(grid_map, start(:, 1:2), goal(:, 1:2));
%path = floor(path);

%% trajectory planning (local)
local_planner = str2func(local_planner_name);
[pose, traj, local_flag] = local_planner(start, goal, "path", path, "map", grid_map);

%% Calculate curvature, get the x and y coordinates of the path/pose
% path is from global planner, which is flipped for some reason
% pose is from local planner

plannerForPlot = "global";
% plannerForPlot = "local";

if plannerForPlot == "global"
    pathEndY = path(1, 1);
    pathEndX = path(1, 2);

    pathReachedGoal = ...
        (pathEndX == gx) && (pathEndY == gy) ...
        || ...
        ceil((pathEndX == gx)) && ceil((pathEndY == gy)) ...
        || ...
        floor((pathEndX == gx)) && floor((pathEndY == gy)) ...
    ;

    if ~pathReachedGoal
        disp('path did not Reach Goal');
        return;
    end
    
    x = flip(path(:, 2)); % colunm 2
    y = flip(path(:, 1)); % column 1

elseif plannerForPlot == "local"
    poseEndY = pose(end, 1);
    poseEndX = pose(end, 2);

    poseReachedGoal = ...
        (poseEndX == gx) && (poseEndY == gy) ...
        || ...
        ceil((poseEndX == gx)) && ceil((poseEndY == gy)) ...
        || ...
        floor((poseEndX == gx)) && floor((poseEndY == gy)) ...
    ;

    if ~poseReachedGoal
        disp('pose did not Reach Goal');
        return;
    end

    x = pose(:, 2); % colunm 2
    y = pose(:, 1); % column 1
end

%% Calculate curvature, parameterize the path and find derivatives
t = linspace(0, 1, length(x)); % Evenly spaced parameter t

% Fit cubic splines for x(t) and y(t)
spline_x = pchip(t, x);
spline_y = pchip(t, y);

% Generate t values for evaluation
t_smooth = linspace(min(t), max(t), length(x));

% First derivatives of x(t) and y(t)
x_1st_derivative = ppval(fnder(spline_x, 1), t_smooth);
y_1st_derivative = ppval(fnder(spline_y, 1), t_smooth);

% Second derivatives of x(t) and y(t)
x_2nd_derivative = ppval(fnder(spline_x, 2), t_smooth);
y_2nd_derivative = ppval(fnder(spline_y, 2), t_smooth);

% Compute curvature using the parametric curvature formula
numerator = x_1st_derivative .* y_2nd_derivative - y_1st_derivative .* x_2nd_derivative;
denominator = (x_1st_derivative.^2 + y_1st_derivative.^2).^(3/2);

curvature = abs(numerator ./ denominator);

%% visualization

% Plot the original path
figure;
plot(x, y, 'o', 'DisplayName', 'Original Points');
hold on;
plot(ppval(spline_x, t_smooth), ppval(spline_y, t_smooth), 'DisplayName', 'Interpolated Path');
xlabel('x');
ylabel('y');
legend('location','southeastoutside');
title('Original Path and Spline Interpolation');

% plot start/goal
plot(sx, sy, '*', 'DisplayName', 'Start');
plot(gx, gy, '*', 'DisplayName', 'Goal');

% Place lengend for the obstacles
plot(0, 0, 's', 'DisplayName', 'Obstacle', ...
    'MarkerEdgeColor','black', 'MarkerFaceColor', 'black' ...
);

% Plot the obstacles on the graph
for i = 1:size(grid_map,1)
    for n = 1:size(grid_map,2)
        if grid_map(i,n) == 2
            plot( ...
                n, i, 's',... 
                'MarkerEdgeColor','black', 'MarkerFaceColor', 'black', ...
                'HandleVisibility', 'off'...
            );
        end
    end
end
hold off;

% Plot new graph with the curvature over time
figure;
hold on;
% plot(t_smooth, curvature, 'DisplayName', 'Curvature');
t_smooth_fitted = min(x) + t_smooth .* (max(x) - min(x));
curvature_fitted = (min(y) + curvature .* (max(y) - min(y)));
plot(t_smooth_fitted, curvature_fitted, 'DisplayName', 'Curvature');
title('Curvature of the Parametric Path');
xlabel('t (parameter)');
ylabel('|Curvature|');
legend;

% Put a datapoint for the curvatures mean on the figure
curvature_fitted_mean = mean(curvature_fitted);
plot(0, curvature_fitted_mean, 'p', 'DisplayName', 'Curvature mean');

hold off;
