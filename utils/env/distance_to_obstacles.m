function distances = distance_to_obstacles(map, pose)
%%
% @file: distance_to_obstacles.m
% @breif: calculate distances to nearest obstacle along path
% @author: Dante van Gemert
% @update: 2024.9.12
% @param map: the map with the obstacles. 1 = free, 2 = obstacle
% @param pose: the array of visited coordinates, of the form [y, x, angle]
%%

obstacle_coords = map_to_points(map);
distances = min(pdist2(obstacle_coords, pose(:,1:2)))';
end

%%
function coords = map_to_points(map)
% @breif: convert map matrix to set of corner points
% @param map: the map with the obstacles. 1 = free, 2 = obstacle

[y, x] = find(map - 1);
coords = [y(:), x(:)];
coords = union(union(union( ...
    coords, ...
    coords + [0 1], "rows"), ...
    coords + [1 0], "rows"), ...
    coords + [1 1], "rows");
end