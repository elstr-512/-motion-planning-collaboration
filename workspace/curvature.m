function [curvature, t_smooth, lengthCost] = curvature(pose)

%% Calculate curvature, get the x and y coordinates of the path/pose
% path is from global planner, which is flipped for some reason
% pose is from local planner


poseEndY = pose(end, 1);
poseEndX = pose(end, 2);

x = pose(:, 2); % colunm 2
y = pose(:, 1); % column 1


%% Calculate curvature, parameterize the path and find derivatives
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

curvature = abs(numerator ./ denominator);

% %% visualization
% 
% % Plot the original path
% figure;
% plot(x, y, 'o', 'DisplayName', 'Original Points', 'Color', [0.6274 0.5098 0.0352]);
% hold on;
% plot(ppval(spline_x, t_smooth), ppval(spline_y, t_smooth), 'DisplayName', 'Interpolated Path');
% xlabel('x');
% ylabel('y');
% legend('location','southeastoutside');
% title('Original Path and Spline Interpolation');
% 
% % plot start/goal
% plot(sx, sy, '*', 'DisplayName', 'Start');
% plot(gx, gy, '*', 'DisplayName', 'Goal');
% 
% % Place lengend for the obstacles
% plot(0, 0, 's', 'DisplayName', 'Obstacle', ...
%     'MarkerEdgeColor','black', 'MarkerFaceColor', 'black' ...
% );
% 
% % Plot the obstacles on the graph
% for i = 1:size(grid_map,1)
%     for n = 1:size(grid_map,2)
%         if grid_map(i,n) == 2
%             plot( ...
%                 n, i, 's',... 
%                 'MarkerEdgeColor','black', 'MarkerFaceColor', 'black', ...
%                 'HandleVisibility', 'off'...
%             );
%         end
%     end
% end
% hold off;
% 
% % Plot new graph with the curvature over time
% figure;
% hold on;
% % plot(t_smooth, curvature, 'DisplayName', 'Curvature');
% t_smooth_fitted = min(x) + t_smooth .* (max(x) - min(x));
% curvature_fitted = (min(y) + curvature .* (max(y) - min(y)));
% plot(t_smooth_fitted, curvature_fitted, 'DisplayName', 'Curvature', 'Color', [0.6274 0.5098 0.0352]);
% title('Curvature of the Parametric Path');
% xlabel('t-parameter scaled to path length');
% ylabel('|Curvature|');
% legend;
% 
% % Put a datapoint for the curvatures mean on the figure
% curvature_fitted_mean = mean(curvature_fitted);
% % plot(0, curvature_fitted_mean, 'p', 'DisplayName', 'Curvature mean');
% 
% hold off;
