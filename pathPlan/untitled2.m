% % 已知折线段的坐标
% offsetTarget2CurrentLane = [4, 5, 8]; % 折线段在x坐标系上的坐标
% sSequcence = [10, 20, 30]; % 折线段在y坐标系上的坐标
% 
% % 已知三次多项式的系数
% coefficients = [0.001, 0.002, 0.001, 0.002]; % 三次多项式的系数
% 
% % 计算曲线1的坐标
% x = sSequcence;
% y = offsetTarget2CurrentLane;
% % 计算曲线2的坐标
% x2 = sSequcence(1):0.5:sSequcence(end); % 参数化曲线2
% y2 = polyval(coefficients, x2);
% % 计算交点坐标
% points1 = [x', y'];
% points2 = [x2', y2'];
% % 计算交点坐标
% [xi, yi] = polyxpoly(x, y, x2, y2);
% 
% % 绘制图形
% figure;
% plot(x, y, 'b-o', 'LineWidth', 2); % 曲线1
% hold on;
% plot(x2, y2, 'r-', 'LineWidth', 2); % 曲线2
% scatter(xi, yi, 'k', 'filled'); % 交点
% legend('Curve 1', 'Curve 2', 'Intersection');
% xlabel('X');
% ylabel('Y');
% title('Intersection of Two Curves');


% % 输入三次多项式系数
% coefficients = [0.001, 0.002, 0.001, 0.002]; % 三次多项式的系数
% 
% % 输入起点和终点坐标
% s_0 = 0; % 起点坐标
% instrusionIntoTargetLaneS = 5; % 终点坐标
% % 输入初速度和加速度
% v_0 = 10; % 初速度
% accerlMean = 1; % 加速度
% 
% % 计算曲线长度
% syms x;
% curveLength = int(sqrt(1 + diff(poly2sym(coefficients), x)^2), s_0, instrusionIntoTargetLaneS);
% instrusionIntoTargetLaneLength = double(curveLength);
% % 计算行驶时间
% if accerlMean == 0
%     time = instrusionIntoTargetLaneLength / v_0;
% else
%     time = (-v_0 + sqrt(v_0^2 + 2 * accerlMean * instrusionIntoTargetLaneLength)) / accerlMean;
% end
% % 计算各个时刻下的位移
% checkTimeGap = 0.2; % 时间间隔（秒）
% t = 0:checkTimeGap:time;
% xCoords=zeros(1,length(t));
% yCoords=xCoords;
% displacement=zeros(1,length(t));
% for i=1:1:length(t) % 时间数组
% displacement(i) = s_0 + v_0 * t(i) + 0.5 * accerlMean * t(i).^2; % 位移数组
% 
% % 计算xCoords
% syms x;
% f = poly2sym(coefficients, x);
% df = diff(f);
% integrand = sqrt(1 + df^2);
% xCoords(i) = double(vpasolve(int(integrand, s_0, x) == displacement(i), x));
% 
% % 计算yCoords
% yCoords(i)  = polyval(coefficients, xCoords(i));
% end
% 
% % 输出结果
% instrusionIntoTargetLaneLength % 曲线长度
% time % 行驶时间
% xCoords
% yCoords
% % 生成x坐标范围
% x = linspace(s_0, instrusionIntoTargetLaneS, 100);
% 
% % 计算y坐标
% y = polyval(coefficients, x);
% 
% % 绘制曲线
% plot(x, y, 'LineWidth', 2);
% xlabel('X');
% ylabel('Y');
% title('Three-Degree Polynomial Curve');


% 输入已知量
s_0 = 0;
coefficients = [0.00, 0.00, 0.00, 0.002]; % 三次多项式的系数
% xCoords = [x1, x2, x3, x4]; % x坐标数组
% yCoords = [y1, y2, y3, y4]; % y坐标数组
accerlMean = 2; % 匀加速度
v_0 = 0; % 初速度
tend = 5; % 行驶时间

% 从 coefficients 数组中获取多项式系数
a3 = coefficients(1);
a2 = coefficients(2);
a1 = coefficients(3);
a0 = coefficients(4);

% 计算多项式的导数
p1 = [3*a3, 2*a2, a1];  % 一次导数
p2 = [2*p1(1), p1(2)];  % 二次导数

% 定义车辆行驶过程中的时间数组
t = linspace(0, tend, 100);

% 计算车辆行驶过程中的位置和速度
x = s_0 + v_0*t + 0.5*accerlMean*t.^2;  % 位置
v = v_0 + accerlMean*t;  % 速度

% 计算车辆行驶过程中的曲率半径
r = abs((1 + (p1(1)*x.^2 + p1(2)*x+p1(3)).^2).^1.5) ./ abs(p2(1)*x + p2(2));

% 计算车辆行驶过程中的横向加速度
a_lat = v.^2 ./ r;

% 获取最大横向加速度并输出
max_a_lat = max(a_lat);
disp(['车辆行驶过程中的最大横向加速度为：', num2str(max_a_lat), ' m/s^2']);
% 绘制4次多项式曲线
figure;
subplot(4, 1, 1);
plot(t, a_lat);
xlabel('时间 (s)');
ylabel('横向加速度 (m/s^2)');
title('车辆行驶过程中的横向加速度');

subplot(4, 1, 2);
plot(t, r);
xlabel('时间 (s)');
ylabel('曲率 (1/m)');
title('车辆行驶过程中的曲率');

subplot(4, 1, 3);
plot(t, v);
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('车辆行驶过程中的速度');

y = a3*x.^3 + a2*x.^2 + a1*x + a0;  % y 坐标

% 绘制车辆行驶的三次多项式曲线
subplot(4, 1, 4);
plot(x, y);
xlabel('x');
ylabel('y');
title('车辆行驶的三次多项式曲线');
