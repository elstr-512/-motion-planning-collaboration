clear all;
addpath(genpath("../utils/"), genpath("../global_planner/"));
clc;

%% explanation
% generates a clear map with a given size, and spawns a window for
% drawing obstacles. When you are done press enter and the grid 
% will be saved to the file "my_gridmap".

%% map setting
map_size = [40, 50];
G = 1;
start = [4, 4];
goal = [36, 46];

% fence around grid
fence = [1:map_size(1), ... 
        map_size(1):map_size(1):map_size(1) * map_size(2), ...
        1:map_size(1):(map_size(1) - 1) * map_size(2), ...
        map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];
obstacle = [fence];

%% create grid or iterate on an old one
%grid_map = generate_grid(map_size, obstacle);
load("my_gridmap.mat");

%% user input
while (1)
    clf; hold on
    
    % plot grid map
    plot_grid(grid_map);

    % plot start and goal
    plot_square(start, map_size, G, "#f00");
    plot_square(goal, map_size, G, "#15c");

    hold off

    % get cursor input
    p = ginput(1);
    if size(p, 1) == 0
        % ENTER means nothing
        break;
    else
        c = floor(p);

        % change state on the grid map
        grid_map(c(2), c(1)) = 3 - grid_map(c(2), c(1));
    end
end

%% save and plot
save my_gridmap grid_map
plot_grid(grid_map);