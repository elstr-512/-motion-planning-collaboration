addpath(genpath("./"));

%% 
% 8 maps
% 5 start & goal
% 5x5x8 = 200 permutations
%

%% 1 <= map_number <= 8 
map_number = 3;

% start and goal pose (y, x, angle)
starts = [
    3,  3; ...
    11, 3; ...
    20, 3; ...
    29, 3; ...
    38, 3; ...
];

goals = [
    3,  38; ...
    11, 38; ...
    20, 38; ...
    29, 38; ...
    38, 38; ...
];

dataFiles = dir("workspace_2\map_space\grid_maps\");
dataFiles = dataFiles(~matches({dataFiles.name}, [".", ".."]), :);

for i = 1:length(dataFiles)
    F = dataFiles(i);
    name = sprintf('workspace_2\\map_space\\grid_maps\\%s', F.name);
    load(name)
    
    map_size = size(grid_map);
    G = 1;
    plot_grid(grid_map);
    hold on;

    for start_i = 1:length(starts)
        for goal_i = 1:length(goals)
        
            start = starts(start_i,:);
            goal  = goals(goal_i,:);
            
            
            plot_square(start, map_size, G, "#f00");
            plot_square(goal, map_size, G, "#15c");
    
            
        end
    end

    hold off;
    if i == map_number
        return;
    end
end