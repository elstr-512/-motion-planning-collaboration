addpath(genpath("./"));

%% 1
% In order to prevent contamination of data
% delete the out\ dir -> workspace_2\run_planners_space\out\
% if changes to the script are made

% if ~isfolder("workspace_2\run_planners_space\out\")
% 
%     mkdir("workspace_2\run_planners_space\out\");
%     mkdir("workspace_2\run_planners_space\out\a_star");
%     mkdir("workspace_2\run_planners_space\out\voronoi_plan");
%     mkdir("workspace_2\run_planners_space\out\dijkstra");
%     mkdir("workspace_2\run_planners_space\out\theta_star");
%     mkdir("workspace_2\run_planners_space\out\gbfs");
% else
%     return;
% end

%% 2
planners = [
    "a_star",       ...
    % "voronoi_plan", ...
    "dijkstra",     ...
    "theta_star",   ...
    "gbfs"
];

%% start & goal can be checked  in tests/startNgoalPoints
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

for map_i = 1:length(dataFiles)
    F = dataFiles(map_i);
    disp(F.name); %%%%% %%%%% %%%%% print
    name = sprintf('workspace_2\\map_space\\grid_maps\\%s', F.name);
    load(name)
 
    map_size = size(grid_map);
    G = 1;

    for planner_i = 1:length(planners)
        planner_name = planners(planner_i);
        disp(planner_name); %%%%% %%%%% %%%%% print

        for start_i = 1:length(starts)
            for goal_i = 1:length(goals)
                start = starts(start_i,:);
                goal  = goals(goal_i,:);

                planner = str2func(planner_name);
                [path, flag, cost, expand] = planner(grid_map, start, goal);
    
    
                if flag == 1
                    save(sprintf(...
                        "workspace_2/run_planners_space/out/%s/map_%02d_start_%02d_goal_%02d.mat", ...
                        planner_name, map_i, start_i, goal_i), ...
                        ...
                        "planner_name", "path", "start", "goal", "grid_map", "cost" ...
                    ) % save
                
                else
                    %%%%% %%%%% %%%%% print
                    fprintf("failed:  %s  map_%02d_start_%02d_goal_%02d\n\n", planner_name, map_i, start_i, goal_i);


                                                    % while (1)
                                                    %         clf; hold on
                                                    % 
                                                    %         % plot grid map
                                                    %         plot_grid(grid_map);
                                                    % 
                                                    %         % plot expand zone
                                                    %         plot_expand(expand, map_size, G, planner_name);
                                                    % 
                                                    %         % plot path
                                                    %         plot_path(path, G);
                                                    % 
                                                    %         % plot start and goal
                                                    %         plot_square(start, map_size, G, "#f00");
                                                    %         plot_square(goal, map_size, G, "#15c");
                                                    % 
                                                    %         % title
                                                    %         title([planner_name, "cost:" + num2str(cost)], 'Interpreter','none');
                                                    % 
                                                    %         hold off
                                                    % 
                                                    %         % get cursor input
                                                    %         p = ginput(1);
                                                    %         if size(p, 1) == 0
                                                    %             % ENTER means nothing
                                                    %             break;
                                                    %         end
                                                    %     end

                    

                end % global_flag == 1

            end % goals
        end % starts


    end % planners
end % dataFiles