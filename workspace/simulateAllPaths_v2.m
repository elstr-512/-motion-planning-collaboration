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
addpath("global_planner/graph_search");
addpath("local_planner");
addpath("utils/env", "utils/data/map");

%% 1
clear all;
clc;

if ~isfolder("out")
    mkdir("out");
    mkdir("out/a_star");
    mkdir("out/voronoi_plan");
    mkdir("out/dijkstra");
end

%% 2
planners = [
    "a_star", ...
    "voronoi_plan", ...
    "dijkstra", ...
];

local_planner_name = "pid_plan";

% start and goal pose (y, x, angle)
starts = [
    4,  4, pi / 2; ...
%    4, 10, pi / 2; ...
    4, 16, pi / 2; ...
    4, 22, pi / 2; ...
%    4, 28, pi / 2; ...
    4, 34, pi / 2; ...
];

goals = [
     4, 46, pi / 2; ...
    % 10, 46, pi / 2; ...
    16, 46, pi / 2; ...
    22, 46, pi / 2; ...
    % 28, 46, pi / 2; ...
    34, 46, pi / 2; ...
];

for map_i = 1:10
    load(sprintf("my_gridmap_%02d.mat", map_i));
    
    for planner_i = 1:length(planners)
        % load environment
        global_planner_name = planners(planner_i);
        map_size = size(grid_map);
        G = 1;
        disp(global_planner_name);
        for start_i = 1:length(starts)
    
            for goal_i = 1:length(goals)
                start = starts(start_i, :);
                goal  = goals(goal_i, :);
    
                % Path planning (global)
                global_planner = str2func(global_planner_name);
                [path, global_flag, cost, expand] = global_planner(grid_map, start(:, 1:2), goal(:, 1:2));
                % path = floor(path); % maybe floor too feed to local
                
                if global_flag == 1
                    % local_planner_name = "none";
                    % save(sprintf("out/path-map_%02d-%s-start_%02d_goal%02d.mat", ...
                    %     map_i, global_planner_name, start_i, goal_i), "global_planner_name", "local_planner_name", "path", "start", "goal", "grid_map")
                    
                    local_planner = str2func(local_planner_name);
                    [pose, traj, local_flag] = local_planner(start, goal, "path", path, "map", grid_map);
                    
                    if local_flag == 1
                        save(sprintf("out/%s/path-map_%02d-%s-%s-start_%02d_goal%02d.mat", ...
                            global_planner_name, ...
                            map_i, local_planner_name, global_planner_name, start_i, goal_i), "global_planner_name", "local_planner_name", "path", "pose", "start", "goal", "grid_map")
                    else
                        disp("global: OK, local: NOT");
                    end

                end
            end
        end
    end
end
