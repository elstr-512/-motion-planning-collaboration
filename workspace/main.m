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
% global_planner_name = "dijkstra";
global_planner_name = "voronoi_plan";

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
        (ceil(pathEndX) == gx) && (ceil(pathEndY) == gy) ...
        || ...
        (floor(pathEndX) == gx) && (floor(pathEndY) == gy) ...
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
        (ceil(poseEndX) == gx) && (ceil(poseEndY) == gy) ...
        || ...
        (floor(poseEndX) == gx) && (floor(poseEndY) == gy) ...
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
plot(x, y, 'o', 'DisplayName', 'Original Points', ...
    'MarkerFaceColor', [0.10196078431372549 1.0 0.0],'MarkerEdgeColor', 'k', ...
    'MarkerSize', 10 ...
    );
hold on;

plot(ppval(spline_x, t_smooth), ppval(spline_y, t_smooth), ...
    'DisplayName', 'Interpolated Path', ...
    'LineWidth',2.0, ...
    'Color', [1.0, 0.0, 0.10196078431372549] ...
);
% D [0.0, 0.6, 1.0]
% V [1.0, 0.0, 0.10196078431372549]


xlabel('x');
ylabel('y');
legend('location','northwest');
title('Original Path and Spline Interpolation');

% plot start/goal
plot(sx, sy, 'square', 'DisplayName', 'Start', 'MarkerSize', 14, ...
    'MarkerFaceColor', [1.0 0.0 0.10196078431372549],'MarkerEdgeColor', 'k');
plot(gx, gy, 'square', 'DisplayName', 'Goal', 'MarkerSize', 14, ...
    'MarkerFaceColor', [1.0  0.4  0.0],'MarkerEdgeColor', 'k' );

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
                'HandleVisibility', 'off', ...
                'MarkerSize', 12 ...
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
plot(t_smooth, curvature, 'DisplayName', 'Voronoi', ...
    'Color', [1.0, 0.0, 0.10196078431372549], ...
    'LineWidth',2.0 );

t_smooth_2 = [0	0.0212765957446809	0.0425531914893617	0.0638297872340426	0.0851063829787234	0.106382978723404	0.127659574468085	0.148936170212766	0.170212765957447	0.191489361702128	0.212765957446809	0.234042553191489	0.255319148936170	0.276595744680851	0.297872340425532	0.319148936170213	0.340425531914894	0.361702127659575	0.382978723404255	0.404255319148936	0.425531914893617	0.446808510638298	0.468085106382979	0.489361702127660	0.510638297872340	0.531914893617021	0.553191489361702	0.574468085106383	0.595744680851064	0.617021276595745	0.638297872340426	0.659574468085106	0.680851063829787	0.702127659574468	0.723404255319149	0.744680851063830	0.765957446808511	0.787234042553192	0.808510638297872	0.829787234042553	0.851063829787234	0.872340425531915	0.893617021276596	0.914893617021277	0.936170212765958	0.957446808510638	0.978723404255319	1]
curvature_2 = [0	0	0	4.00000000000000	0	0	0	0	0	0	0	0	0.707106781186547	0	0	0	0	4.00000000000001	0	0.707106781186547	0	0	0	0	0	4.00000000000000	0	0	0	0	0	0	0	0.707106781186550	0	0	0	0	0	0	0	0	3.99999999999998	0	0	0	0	0];
plot(t_smooth_2, curvature_2, 'DisplayName', 'Dijkstra', ...
    'Color', [0.0, 0.6, 1.0], ...
    'LineWidth',2.0 );

title('Curvature of the Parametric Path');
xlabel('t');
ylabel('|Curvature|');
legend;



% Put a datapoint for the curvatures mean on the figure
% curvature_fitted_mean = mean(curvature_fitted);
% plot(0, curvature_fitted_mean, 'p', 'DisplayName', 'Curvature mean');

hold off;

