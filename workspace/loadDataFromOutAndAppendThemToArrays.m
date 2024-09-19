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
            %% TRY to get curvature and path length
            x = pose(:, 2); % colunm 2
            y = pose(:, 1); % column 1
            % Calculate curvature, parameterize the path and find derivatives
            t = linspace(0, 1, length(x)); % Evenly spaced parameter t
            
            % Fit cubic splines for x(t) and y(t)
            spline_x = pchip(t, x);
            spline_y = pchip(t, y);
            
            % Generate t values for evaluation
            t_smooth = linspace(min(t), max(t), length(x));
            
            lengthCost = arclength(ppval(spline_x, t_smooth), ppval(spline_y, t_smooth), 'pchip');
            
            % First derivatives of x(t) and y(t)
            x_1st_derivative = ppval(fnder(spline_x, 1), t_smooth);
            y_1st_derivative = ppval(fnder(spline_y, 1), t_smooth);
            
            % Second derivatives of x(t) and y(t)
            x_2nd_derivative = ppval(fnder(spline_x, 2), t_smooth);
            y_2nd_derivative = ppval(fnder(spline_y, 2), t_smooth);
            
            % Compute curvature using the parametric curvature formula
            numerator = x_1st_derivative .* y_2nd_derivative - y_1st_derivative .* x_2nd_derivative;
            denominator = (x_1st_derivative.^2 + y_1st_derivative.^2).^(3/2);
            
            curvatures = abs(numerator ./ denominator)';

            % Compute distance to obstacles
            distances = distance_to_obstacles(grid_map, pose);
            
            %% Append data to arrays
            all_lengths(file_i) = lengthCost;
            all_curvatures{file_i} = curvatures;
            all_distances{file_i} = distances;
        end
        % M = mean(A, 2);
    end
    
    %% Save lengthCostArray & curvCostArray for plotting 
    save(sprintf("out/data_%s.mat", planners(planner)), ...
        "all_lengths", "all_curvatures", "all_distances" ...
    )

end