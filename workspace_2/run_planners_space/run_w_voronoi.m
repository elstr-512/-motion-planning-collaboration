addpath(genpath("./"));

%% 1
% In order to prevent contamination of data
% delete the out\ dir -> workspace_2\run_planners_space\out\
% if changes to the script are made

if ~isfolder("workspace_2\run_planners_space\out\")

    mkdir("workspace_2\run_planners_space\out\");
    mkdir("workspace_2\run_planners_space\out\a_star");
    mkdir("workspace_2\run_planners_space\out\voronoi_plan");
    mkdir("workspace_2\run_planners_space\out\dijkstra");
    mkdir("workspace_2\run_planners_space\out\theta_star");
    mkdir("workspace_2\run_planners_space\out\gbfs");
end

%% 2
planners = [
    "a_star",       ...
    "dijkstra",     ...
    "theta_star",   ...
    "gbfs"
];

% start & goal that works well with voronoi
starts = [
    20,  5; ...
    29,  3; ...
    29,  5; ...
    38,  3; ...
    38,  4; ...
    38,  5; ...
    11,  3; ...
    11,  4; ...
    20,  3; ...
    20,  5; ...
    29,  3; ...
    29,  4; ...
    11,  3; ...
    11,  4; ...
    20,  24; ...
    29,  5; ...
];

goals = [
     3,  35; ...
    19,  37; ...
    14,  38; ...
    11,  36; ...
    20,  36; ...
    31,  37; ...
    18,  36; ...
    21,  37; ...
    31,  38; ...
    11,  36; ...
    22,  38; ...
    33,  38; ...
    3,  35; ...
    6,  36; ...
    12,  35; ...
    3,  38; ...
    4,  38; ...
    12,  36; ...
    9,  36; ...
    13,  36; ...
    14,  35; ...
    3,  35; ...
    4,  35; ...
    7,  36; ...
    10,  36; ...
    10,  35; ...
    25,  35; ...
    30,  35; ...
    29,  36; ...
    25,  35; ...
    27,  25; ...
    28,  35; ...
];

%% start & goal can be checked  in tests/startNgoalPoints


dataFiles = dir("workspace_2\map_space\grid_maps\");
dataFiles = dataFiles(~matches({dataFiles.name}, [".", ".."]), :);

for map_i = 1:length(dataFiles)

    F = dataFiles(map_i);
    disp(F.name); %%%%% %%%%% %%%%% print
    name = sprintf('workspace_2\\map_space\\grid_maps\\%s', F.name);
    load(name)
 
    map_size = size(grid_map);
    G = 1;

    for start_i = 1:length(starts)
        start = starts(start_i,:);

        for goal_i = 1:length(goals)

            goal  = goals(goal_i,:);    
            planner_name = "voronoi_plan";
            planner = str2func(planner_name);
            [path, flag, cost, expand] = planner(grid_map, start, goal);
                
                % if voronoi works, save & run the other planners
                if flag == 1
                    save(sprintf(...
                        "workspace_2/run_planners_space/out/%s/map_%02d_start_%02d%02d_goal_%02d%2d.mat", ...
                        planner_name, map_i, start(1), start(2), goal(1), goal(2)), ...
                        ...
                        "planner_name", "path", "start", "goal", "grid_map", "cost" ...
                    ) % save
                
                    % since voronio worked run the other planners
                    for planner_i = 1:length(planners)
                        planner_name = planners(planner_i);
                        planner = str2func(planner_name);
                        [path, flag, cost, expand] = planner(grid_map, start, goal);
                        if flag == 1
                            save(sprintf(...
                                "workspace_2/run_planners_space/out/%s/map_%02d_start_%02d%02d_goal_%02d%2d.mat", ...
                                planner_name, map_i, start(1), start(2), goal(1), goal(2)), ...
                                ...
                                "planner_name", "path", "start", "goal", "grid_map", "cost" ...
                            ) % save
                        end % flag == 1

                    end % planners

                end % flag == 1

        end % goals

    end % starts


    
end % dataFiles

sucsess = dir("workspace_2\run_planners_space\out\voronoi_plan");
disp(length(sucsess));