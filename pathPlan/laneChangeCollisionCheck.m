clear all;
close all;

% 定义横轴t和纵轴s的范围
t = linspace(0, 4);
s = linspace(0, 70);

figure('Position', [0, 100, 900, 100+400]) % 创建一个指定大小的图形窗口

% 画第一条直线
line1_x = [0, 4];
line1_y = [0, 70];

% 画第二条二次曲线
line2_x = linspace(0, 1.5);
line2_y = 10 + (40-10)/(1.5-0)^2 * (line2_x-0).^2;

% 画第三条曲线
line3_x = linspace(0.5, 3.5);
line3_y = 30 + (70-30)/(3-0.5)^2 * (line3_x-0.5).^2;

% 画第四条曲线
line4_x = linspace(2.5, 4);
line4_y = 30 + (60-30)/(1.5-0)^2 * (line4_x-2.5).^2;

% 画第五条直线
line5_x = [0, 4];
line5_y = [30, 30];

% 画第六条直线
line6_x = [0, 4];
line6_y = [40, 40];
line7_x = [0, 3];
line7_y = [70, 70];
line8_x = [2.5, 4];
line8_y = [0, 0];

x1_fill = [line4_x, fliplr(line8_x)];
y1_fill = [line4_y, fliplr(line8_y)];
fill(x1_fill, y1_fill, [0.9, 0.9, 0.9], 'LineStyle', 'none');
hold on;

x2_fill = [line2_x, fliplr(line7_x)];
y2_fill = [line2_y, fliplr(line7_y)];
fill(x2_fill, y2_fill, [0.9, 0.9, 0.9], 'LineStyle', 'none');
hold on;

x3_fill = [line3_x, fliplr(line7_x)];
y3_fill = [line3_y, fliplr(line7_y)];
h1=fill(x3_fill, y3_fill, [0.9, 0.9, 0.9], 'LineStyle', 'none');
hold on;

% 绘制图像
h2=plot(line1_x, line1_y, 'r');
h3=plot(line2_x, line2_y, 'g');
h4=plot(line3_x, line3_y, 'b');
h5=plot(line4_x, line4_y, 'm');
h6=plot(line5_x, line5_y,'--');
h7=plot(line6_x, line6_y,'--');


xlabel('t(s)');
ylabel('s(m)');

legend([h1, h2, h3, h4, h5, h6, h7],...,
    'restricted area','trajectory of ego car', 'trajectory of c', 'trajectory of b', 'trajectory of a', ...,
    'intrusion into the target lane', 'departure from the current lane','Location','northwest','FontSize',10);

% x_fill = line2_x;
% y_fill = line2_y;
% fill(x_fill, y_fill, 'gray');
xlim([0, 3]) % 设置 x 轴的范围为 0 到 100
ylim([0, 70]) % 设置 x 轴的范围为 0 到 100


hold off;
