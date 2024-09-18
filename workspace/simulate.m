addpath("global_planner/graph_search");
addpath("local_planner");
addpath("utils/env", "utils/data/map");

clear all;
clc;

if ~isfolder("out")
    mkdir("out");
end

%% Initialization

% amount of map update steps inbetween start and goal
n_steps = 1;

% combinations of global and local planners
planners = [
    "a_star"        "apf_plan"; ...
    "voronoi_plan"  "apf_plan"; ...
    "dijkstra"      "apf_plan"; ...
    "",             "dwa_plan"; ...
];

% start and goal pose (y, x, angle)
starts = [
    2,  4, pi / 2; ...
    2, 10, pi / 2; ...
    2, 16, pi / 2; ...
    2, 22, pi / 2; ...
    2, 28, pi / 2; ...
    2, 34, pi / 2; ...
];

goals = [
    48,  4, pi / 2; ...
    48, 10, pi / 2; ...
    48, 16, pi / 2; ...
    48, 22, pi / 2; ...
    48, 28, pi / 2; ...
    48, 34, pi / 2; ...
];

% load environment
grid_map = load("gridmap_ships_01.mat");
map_size = size(grid_map);
G = 1;

%% Planning functions

function path = plan_global(planner_name, map, start, goal)
    planner = str2func(planner_name);
    [path, ~, ~, ~] = planner(map, start(:, 1:2), goal(:, 1:2));
end

function pose = plan_local(planner_name, path, map, start, goal)
    planner = str2func(planner_name);
    [pose, ~, ~] = planner(start, goal, "path", path, "map", map);
end

%% Run simulation

poses = cell(length(planners), length(starts), length(goals));

% loop over all planners
for planner_i = 1:size(planners, 1)
    planner = planners(planner_i,:);
    % loop over all combinations of starting and ending poses
    for start_i = 1:size(starts, 1)
        for goal_i = 1:size(goals, 1)
            start = starts(start_i,:);
            goal = goals(goal_i,:);
            pose = [];
            fprintf("planner: %12s / %-8s    start: %.2f %.2f %.2f    goal: %.2f %.2f %.2f\n", planner, start, goal);
            for step_i = 1:n_steps
                grid_map = load(sprintf("gridmap_ships_%02d.mat", step_i));
                if planner(1) ~= ""
                    rounded_start = [round(start(1:2)), start(3)];
                    path = plan_global(planner(1), grid_map, rounded_start, goal);
                end
                new_pose = plan_local(planner(2), path, grid_map, start, goal);
                cutoff = min(length(new_pose), floor(length(new_pose) / (n_steps - step_i + 1)));
                start = new_pose(cutoff, :);
                pose = [pose; new_pose(1:cutoff, :)];
            end

            start = starts(start_i,:); % reset start variable to initial value for saving
            save(sprintf("out/pose-%s-%s-%.2f_%.2f_%.2f-%.2f_%.2f_%.2f.mat", planner, start, goal), "pose", "start", "goal")
            poses{planner_i, start_i, goal_i} = pose;
        end
    end
end