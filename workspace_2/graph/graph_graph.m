%% imports
addpath(genpath("./"));

%% Test data


%% Load data
function [curvatures, lengths, distances] = load_data(file)
    all_data = load(file);
    curvatures = vertcat(all_data.all_curvatures{:});
    curvatures = curvatures(curvatures > 0);
    curvatures = curvatures(curvatures < 6);
    lengths = vertcat(all_data.all_lengths(:));
    distances = vertcat(all_data.all_distances{:});
end

[curvatures_a_star, lengths_a_star, distances_a_star] = load_data("workspace_2/run_planners_space/out/data_a_star.mat");
[curvatures_dijkstra, lengths_dijkstra, distances_dijkstra] = load_data("workspace_2/run_planners_space/out/data_dijkstra.mat");
[curvatures_voronoi_plan, lengths_voronoi_plan, distances_voronoi_plan] = load_data("workspace_2/run_planners_space/out/data_voronoi_plan.mat");
[curvatures_gbfs, lengths_gbfs, distances_gbfs] = load_data("workspace_2/run_planners_space/out/data_gbfs.mat");
[curvatures_theta_star, lengths_theta_star, distances_theta_star] = load_data("workspace_2/run_planners_space/out/data_theta_star.mat");

%% Save to table data in workspace_2/graph/table_data/
% https://tableconvert.com/csv-to-markdown
% use this to get pretty tables
met_n = {'curvatures_', 'lengths_', 'distances_'};
alg_n = {'a_star', 'dijkstra', 'voronoi_plan', 'gbfs', 'theta_star'};
dat_n = ...
{ ...
    curvatures_a_star, curvatures_dijkstra, curvatures_voronoi_plan, curvatures_gbfs, curvatures_theta_star; ...
    lengths_a_star, lengths_dijkstra, lengths_voronoi_plan, lengths_gbfs, lengths_theta_star; ...
    distances_a_star, distances_dijkstra, distances_voronoi_plan, distances_gbfs, distances_theta_star; ...
};

for m = 1:length(met_n)
    data = {alg_n{1}, round(mean(dat_n{m, 1}), 2), round(median(dat_n{m, 1}), 2); ...
            alg_n{2}, round(mean(dat_n{m, 2}), 2), round(median(dat_n{m, 2}), 2); ...
            alg_n{3}, round(mean(dat_n{m, 3}), 2), round(median(dat_n{m, 3}), 2); ...
            alg_n{4}, round(mean(dat_n{m, 4}), 2), round(median(dat_n{m, 4}), 2); ...
            alg_n{5}, round(mean(dat_n{m, 5}), 2), round(median(dat_n{m, 5}), 2); ...
    };
    
    headers = {'AlgName', 'Mean', 'Median'};
    T = table(data(:,1), data(:,2), data(:,3), ...
        'VariableNames', headers ...
    );
    
    % Export as Excel fil
    excelFile = sprintf('workspace_2/graph/table_data/%stable_data.xlsx', met_n{m});
    writetable(T, excelFile, 'Sheet', 'Sheet1');
    % Export as CSV file
    csvFile = sprintf('workspace_2/graph/table_data/%stable_data.csv', met_n{m});
    writetable(T, csvFile);
end


%% Combine data
function [data, idx] = combine_data(all_data)
    data = vertcat(all_data{:});
    lengths = cellfun(@length, all_data);
    idx = [];
    for i = 1:length(lengths)
       idx = [idx; repmat(i, lengths(i), 1)];
    end
end

[curvatures, curvatures_idx] = combine_data({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan; curvatures_gbfs; curvatures_theta_star});
[lengths, lengths_idx] = combine_data({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan; lengths_gbfs; lengths_theta_star});
[distances, distances_idx] = combine_data({distances_a_star; distances_dijkstra; distances_voronoi_plan; distances_gbfs; distances_theta_star});

%% Plotting visuals
algorithms = {'A*', 'Dijkstra', 'Voronoi', 'Theta*'};
colororders = [
    0.10196078431372549, 1.0, 0.0; 
    0.0, 0.6, 1.0; 
    1.0, 0.0, 0.10196078431372549; 
    0.4, 0.0, 1.0; 
    1.0 , 0.4 , 0.0; 
];



%% Box chart length
% https://se.mathworks.com/help/matlab/ref/boxchart.html
% usecases: time/distance, collision avoidance
% Prepare data for box chart

function make_plot(data, colororders, algorithms, y_label, name, alpha, type, plotmean)
    figure; hold on; colororder(colororders);
    set(gcf,'position',[0,0,800,600]);
    % Customize colors for each algorithm
    for i = 1:length(data)
        if type == "box"
            boxchart(repmat(i, length(data{i}), 1), data{i}, 'boxfacealpha', 0.6);
        elseif type == "swarm"
            swarmchart(repmat(i, length(data{i}), 1), data{i}, 'filled','MarkerFaceAlpha',alpha);
        end
    end
    for i = 1:length(data)
         if plotmean
             % plot mean
             plot(i, mean(data{i}), 'o', ...
                 'MarkerFaceColor', 'w','MarkerEdgeColor', 'k', ...
                 'MarkerSize', 8 ...
             );
         end

         % plot black square for median since it disappeared on some boxes
         % when they overlap with the edge
         plot(i, median(data{i}), 'square', ...
             'MarkerFaceColor', 'k','MarkerEdgeColor', 'k', ...
             'MarkerSize', 8 ...
         );
    end
    % make every plot start at (0,0)
    plot(0, 0, '.', ...
             'MarkerFaceColor', 'w','MarkerEdgeColor', 'w', ...
             'MarkerSize', 1 ...
         );

    hold off;
    
    % Add labels and legend
    xticks(1:length(data));
    xticklabels(algorithms);
    ylabel(y_label);
    title(name);
    if type == "box"
        legend(algorithms, 'Location', 'northeastoutside', 'FontSize', 16);
    elseif type == "swarm"
        [BL, BLicons] = legend(algorithms, 'Location', 'northeastoutside', 'FontSize', 16);
        PatchInLegend = findobj(BLicons, 'type', 'patch');
        set(PatchInLegend, 'facea', 0.5);
    end
end
%%
make_plot({curvatures_a_star; curvatures_dijkstra; curvatures_voronoi_plan; curvatures_theta_star}, ...
    colororders, algorithms, "Curvature", "Algorithm Performance - Curvature Comparison", 0.05, "swarm", false);
%%
make_plot({lengths_a_star; lengths_dijkstra; lengths_voronoi_plan; lengths_theta_star}, ...
    colororders, algorithms, "Length", "Algorithm Performance - Path Length Comparison", 1.0, "box", false);
%%
make_plot({distances_a_star; distances_dijkstra; distances_voronoi_plan; distances_theta_star}, ...
    colororders, algorithms, "Distance", "Algorithm Performance - Distance to Obstacles Comparison", 0.2, "box", true);
