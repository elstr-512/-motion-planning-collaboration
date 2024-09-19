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
addpath("out/");

%% 1
planners = [
    "a_star", ...
    "voronoi_plan", ...
    "dijkstra", ...
];

% Specify the folder where the files live.
for planner = 1:length(planners)
    folder = sprintf("out/%s", planners(planner));

    addpath(folder);
    dataFiles = dir(folder);
    dataFiles = dataFiles(~matches({dataFiles.name}, [".", ".."]), :);

    all_curvatures = cell(length(dataFiles),1);
    all_lengths    = zeros(length(dataFiles),1);
    all_distances  = cell(length(dataFiles),1);
    
    for file_i = 1:length(dataFiles)
        file = dataFiles(file_i);
        fprintf("%4d / %-4d    %s\n", file_i, length(dataFiles), file.name)
    
        dataStruct = load(file.name);
        global_planner_name = dataStruct.global_planner_name;
        local_planner_name  = dataStruct.local_planner_name;
        grid_map            = dataStruct.grid_map;
        % path                = dataStruct.path;
        pose                = dataStruct.pose;
        
        try 
            [curvatures, ~, lengthCost] = curvature(pose);
            distances = distance_to_obstacles(grid_map, pose);
            
            %% Append data to arrays
            all_lengths(file_i) = lengthCost;
            all_curvatures{file_i} = curvatures';
            all_distances{file_i} = distances;
        end
        % M = mean(A, 2);
    end
    
    %% Save lengthCostArray & curvCostArray for plotting 
    save(sprintf("out/data_%s.mat", planners(planner)), ...
        "all_lengths", "all_curvatures", "all_distances" ...
    )

end