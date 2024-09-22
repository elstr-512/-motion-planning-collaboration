addpath(genpath("./"));
%% 1
dataFiles = dir("workspace_2\map_space\excel_maps\maps_csv");
dataFiles = dataFiles(~matches({dataFiles.name}, [".", ".."]), :);

save_folder = "workspace_2\map_space\grid_maps\";

for i = 1:length(dataFiles)
    F = dataFiles(i);
    N = F.name;
    T   = readtable(N);
    grid_map = table2array(T);

    save_name = sprintf('%smap_%d.mat',save_folder, i);
    save(save_name, "grid_map");

    plot_grid(grid_map);
end