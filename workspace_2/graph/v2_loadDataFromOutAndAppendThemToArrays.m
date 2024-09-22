%% imports
addpath(genpath("./"));

%% 1
planners = [
    "a_star",       ...
    "dijkstra",     ...
    "voronoi_plan",      ...
    "theta_star",   ...
    "gbfs"
];

% Specify the folder where the files live.
for planner = 1:length(planners)
    folder = sprintf("workspace_2/run_planners_space/out/%s", planners(planner));

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

        planner_name        = dataStruct.planner_name;
        grid_map            = dataStruct.grid_map;
        path                = dataStruct.path;
        cost                = dataStruct.cost;
        try 
            [curvatures, ~, ~] = curvature(path);
            distances = distance_to_obstacles(grid_map, path);
            
            %% Append data to arrays
            all_lengths(file_i) = cost;
            all_curvatures{file_i} = curvatures';
            all_distances{file_i} = distances;
        end
        % M = mean(A, 2);
    end
    
    %% Save lengthCostArray & curvCostArray for plotting )
    save(sprintf("workspace_2/run_planners_space/out/data_%s.mat", planners(planner)), ...
        "all_lengths", "all_curvatures", "all_distances" ...
    )

end