clear all;
close all;


% 定义时间间隔和初始条件
dt = 0.5;               % 时间间隔为0.5秒
initial_velocity = 9.5;  % 初始速度为20 m/s
dtPoint=0.1;
a_min_com=-2;
a_min=-4;
a_max_com=1.25;
a_max=2.5;

% 定义每段的加速度
accelerations = [a_max, a_max_com , a_max_com, a_min_com, a_min_com, a_min, a_min, a_min, a_min, a_min];

% 计算每段的位移
displacements = zeros(size(accelerations));
vend = zeros(size(accelerations));

for i = 1:length(accelerations)
    if i == 1
        displacements(i) = initial_velocity * dt + 0.5 * accelerations(i) * dt^2;
    else
        displacements(i) = displacements(i-1) + initial_velocity * dt + 0.5 * accelerations(i) * dt^2;
    end
    initial_velocity = initial_velocity + accelerations(i) * dt;
    vend(i) = initial_velocity;
end

time = 0:dtPoint:(length(accelerations) * dt);
displacement_points = zeros(size(time));
initial_velocity=9.5;
for i = 1:length(time)
    segment_index = max(1,ceil(time(i) / dt));  % 找到当前时间所处的分段
    if i == 1
        displacement_points(i) = initial_velocity * time(i) + 0.5 * accelerations(segment_index) * time(i)^2;
    else
        displacement_points(i) = displacement_points(i-1) + initial_velocity * dtPoint + 0.5 * accelerations(segment_index) * dtPoint^2;
%         displacements(segment_index-1) +...,
%             (initial_velocity * (time(i)-(segment_index-1)*dt)) + 0.5 * accelerations(segment_index) * (time(i)-(segment_index-1)*dt)^2;
    end
    initial_velocity = initial_velocity + accelerations(segment_index) * dtPoint;
end

% 绘制位移-时间曲线
% time = 0:dt:(length(accelerations) * dt);

figure('Position', [0, 100, 900, 100+400]) % 创建一个指定大小的图形窗口


% 绘制方框
% h1=rectangle('Position', [1, 0, 0.5, 9], 'FaceColor', [0.9, 0.9, 0.9]);
rectangle('Position', [2, 15, 0.5, 5], 'EdgeColor', [0.8, 0.8, 0.8],'FaceColor', [0.8, 0.8, 0.8]);
rectangle('Position', [3.5, 30, 1, 5], 'EdgeColor', [0.8, 0.8, 0.8],'FaceColor', [0.8, 0.8, 0.8]);
rectangle('Position', [2.5, 24, 1.5, 2],'EdgeColor', [0.8, 0.8, 0.8], 'FaceColor', [0.8, 0.8, 0.8]);
rectangle('Position', [0.5, 24, 1, 5], 'EdgeColor', [0.8, 0.8, 0.8],'FaceColor', [0.8, 0.8, 0.8]);

% 绘制线段
h1=line([1, 1.5], [4.5, 4.5], 'Color', [0.8, 0.8, 0.8], 'LineWidth', 50);

h2=line([2, 5], [45, 45], 'Color', [0.6, 0.6, 0.6], 'LineWidth', 3);
hold on;

h3=plot(time, displacement_points, 'b.-');

xlabel('t (s)');
ylabel('s (m)');

title('Displacement-Time Curve of MCTS Segmented Motion');
grid on;

% 标注每一段的加速度
for i = 1:length(accelerations)
    segment_start_time = (i-1) * dt;
    segment_end_time = i * dt;
    segment_mid_time = (segment_start_time + segment_end_time) / 2;
    segment_acceleration = accelerations(i);
    text(segment_mid_time, displacements(i)+1, sprintf('a=%.2f', segment_acceleration), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(segment_mid_time, displacements(i)+3, sprintf('v=%.2f', vend(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
% 每隔5个点标红
hold on;
for i = 1:5:length(displacement_points)
    plot(time(i), displacement_points(i), 'ro');
end

% 绘制线段（1，0）到（1，6.3），设置为虚线
x1 = [1 1];
y1 = [0 6.3];
plot(x1, y1, '--r');

hold on; % 保持图形窗口打开

% 绘制线段（1，6.3）到（1.5，6.3），设置为虚线
x2 = [1 1.5];
y2 = [8.5 8.5];
plot(x2, y2, '--r');

% 绘制线段（1.5，6.3）到（1.5，0），设置为虚线
x3 = [1.5 1.5];
y3 = [6.3 0];
plot(x3, y3, '--r');

% 绘制线段（1.5，0）到（2，0），设置为虚线
x4 = [1.5 2];
y4 = [0 0];
plot(x4, y4, '--r');

% 绘制线段（2，0）到（2，20），设置为虚线
x5 = [2 2];
y5 = [0 20];
plot(x5, y5, '--r');

% 绘制线段（2，20）到（2.5，20），设置为虚线
x6 = [2 2.5];
y6 = [20 20];
plot(x6, y6, '--r');

% 绘制线段（2.5，20）到（2.5，26），设置为虚线
x7 = [2.5 2.5];
y7 = [20 26];
plot(x7, y7, '--r');

% 绘制线段（2.5，26）到（3.5，26），设置为虚线
x8 = [2.5 3.5];
y8 = [26 26];
plot(x8, y8, '--r');

% 绘制线段（3.5，26）到（3.5，35），设置为虚线
x9 = [3.5 3.5];
y9 = [26 35];
plot(x9, y9, '--r');

% 绘制线段（3.5，35）到（4.5，35），设置为虚线
x10 = [3.5 4.5];
y10 = [35 35];
plot(x10, y10, '--r');

% 绘制线段（4.5，35）到（4.5，0），设置为虚线
x11 = [4.5 4.5];
y11 = [35 0];
h4=plot(x11, y11, '--r');

% 绘制线段（0.5，50）到（0.5，24），设置为蓝色虚线
x1 = [0.5 0.5];
y1 = [50 24];
h5=plot(x1, y1, '--b');

hold on; % 保持图形窗口打开

% 绘制线段（0.5，24）到（1.5，24），设置为蓝色虚线
x2 = [0.5 1.5];
y2 = [24 24];
plot(x2, y2, '--b');

% 绘制线段（1.5，24）到（1.5，50），设置为蓝色虚线
x3 = [1.5 1.5];
y3 = [24 50];
plot(x3, y3, '--b');

% 绘制线段（2，50）到（2，44），设置为蓝色虚线
x4 = [2 2];
y4 = [50 44.8];
plot(x4, y4, '--b');

% 绘制线段（2，44）到（5，44），设置为蓝色虚线
x5 = [2 5];
y5 = [44.8 44.8];
plot(x5, y5, '--b');

hold off;
legend([h1, h2, h3, h4, h5],'restricted area by obstacles','restricted area by traffic light','coarse motion curve','lower limit','upper limit','Location','northwest','FontSize',10);

xlim([0, 5]) % 设置 x 轴的范围为 0 到 100
ylim([0, 50]) % 设置 x 轴的范围为 0 到 100

