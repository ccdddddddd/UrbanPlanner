% 定义长方形的四个顶点坐标
rect1 = [1, 1; 1, 2; 2, 2; 2, 1];
rect2 = [3, 3; 3, 4; 4, 4; 4, 3];
rect3 = [1, 4; 1, 5; 2, 5; 2, 4];
% 其他长方形定义...
rectangles = {rect1, rect2, rect3}; % 将所有长方形放入一个cell数组中，可以根据实际情况修改


% 定义t=tx直线的横坐标
tx = 1.5;

% 初始化交点数组
crossPoints = [];

% 遍历每个长方形
for i = 1:length(rectangles)
    rect = rectangles{i};

    % 计算直线与长方形的交点
    crossPoint = [];
    for j = 1:4
        % 获取相邻的两个顶点
        p1 = rect(j, :);
        p2 = rect(mod(j, 4) + 1, :);

        % 判断直线与边的交点是否存在
        if (p1(1) <= tx && p2(1) >= tx) || (p1(1) >= tx && p2(1) <= tx)
            % 计算交点的纵坐标
            if p2(1) - p1(1)~=0
                s = p1(2) + (tx - p1(1)) * (p2(2) - p1(2)) / (p2(1) - p1(1));
                % 添加交点到数组中
                crossPoint = [crossPoint,s];
            end

        end
    end

    % 如果存在交点，则添加到crossPoints数组中
    if ~isempty(crossPoint)
        crossPoints = [crossPoints; [sort(crossPoint),0]];
    end
end

% 显示结果
disp(crossPoints);


% % 长方形顶点坐标
% rectangles = [
%     1, 2, 3, 2;
%     4, 5, 6, 5;
%     7, 8, 9, 8
% ];
%
% % 时间和位移范围
% time = 0:0.1:1;
% s_range = min(rectangles(:, [1, 3])):0.1:max(rectangles(:, [2, 4]));
%
% % 计算不可通行区域
% obstacles = zeros(length(s_range), length(time));
% for i = 1:length(rectangles)
%     s_min = min(rectangles(i, [1, 3]));
%     s_max = max(rectangles(i, [2, 4]));
%     t_min = min(rectangles(i, [5, 7]));
%     t_max = max(rectangles(i, [6, 8]));
%     [s_idx_min, s_idx_max] = find_range_idx(s_range, s_min, s_max);
%     [t_idx_min, t_idx_max] = find_range_idx(time, t_min, t_max);
%     obstacles(s_idx_min:s_idx_max, t_idx_min:t_idx_max) = 1;
% end
%
% % 将不可通行区域表示为数组
% obstacle_ranges = zeros(length(time), 2);
% for i = 1:length(time)
%     obstacle_s_ranges = find_ranges(s_range(obstacles(:, i) == 1));
%     obstacle_ranges(i, :) = obstacle_s_ranges(:)';
% end
%
% % 辅助函数，查找范围的索引
% function [idx_min, idx_max] = find_range_idx(range, val_min, val_max)
%     idx_min = find(range >= val_min, 1);
%     idx_max = find(range <= val_max, 1, 'last');
% end
%
% % 辅助函数，查找范围
% function ranges = find_ranges(indices)
%     ranges = [];
%     if ~isempty(indices)
%         diff_indices = diff(indices);
%         range_starts = find(diff_indices > 1) + 1;
%         range_ends = [range_starts(2:end) - 1; length(indices)];
%         ranges = [indices(range_starts), indices(range_ends)];
%     end
% end
