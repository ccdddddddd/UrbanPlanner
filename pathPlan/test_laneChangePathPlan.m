clear all;
close all;
v0=20;
%% 生成多条曲线
tmax=6;
amax=1.5;
amin=-2;
alateralmax=3;
yend=3.2;
w=0.5;
vmax=20;
vmin=6;
r=5;
% v0+amax*tmax
% min(vmax,v0+amax*tmax)
xendmax=max(sqrt(r.^2-(r-yend).^2),sqrt(((v0+min(vmax,v0+amax*tmax))/2*tmax).^2-yend.^2));
vendmin=max(vmin,v0+amin*tmax);
vendmax=min(vmax,v0+amax*tmax);
interal=(vendmax-vendmin)/round((vendmax-vendmin)/2);
tendList=zeros(1,round((vendmax-vendmin)/2)+1);
xendList=zeros(1,round((vendmax-vendmin)/2)+1);
vendList=vendmin:interal:vendmax;
validList=zeros(1,round((vendmax-vendmin)/2)+1);
figure('Position', [0, 100, 1200, 100+400]) % 创建一个指定大小的图形窗口
xlabel('x') % 设置 x 轴标签
ylabel('y') % 设置 y 轴标签
%% 动力学检验
for i=1:1:round((vendmax-vendmin)/2)+1
    vend=vendList(i);
    xend=(4*w*(6*vend^2*yend/alateralmax)^2/((1-w)/xendmax))^0.2;
    % 定义积分区间
    a = 0; % 曲线的起点横坐标
    b = xend; % 曲线的终点横坐标
    % 定义被积函数
    f = @(x) sqrt(1 + (6*x*yend/xend.^2 - 6*yend/xend.^3*x.^2).^2);
    % 计算积分
    L = integral(f, a, b);
    tend=L/((v0+vend)/2); % tend=sqrt(xend^2+yend^2)/((v0+vend)/2);
    accerlMean=(vend-v0)/tend;
    tendList(i)=tend;
    xendList(i)=xend;
    if accerlMean>amin && accerlMean<amax
        validList(i)=1;
    end
    x = 0:0.1:xend; % 生成 x 的取值范围，间隔为0.1
    y = (3*yend/(xend^2))*(x.^2) - (2*yend/(xend^3))*(x.^3); % 计算 y 的值
    if validList(i) == 1 % 如果 validList 为 1，则绘制实线
        plot(x, y, 'LineWidth', 2) % 绘制实线
    else % 否则，绘制虚线
        plot(x, y, '--', 'LineWidth', 2) % 绘制虚线
    end
    % 标注 xend、yend 和 tend 的值
%     text(xend*0.7, yend*0.8, ['xend = ', num2str(xend)])
%     text(xend*0.7, yend*0.7, ['yend = ', num2str(yend)])
%     text(xend*0.7, yend*0.6, ['tend = ', num2str(tend)])
%     text(xend*0.9, yend*1.02, ['Xend = ', num2str(xend, '%.1f')])
%     text(xend*0.7, yend*0.95, ['yend = ', num2str(yend, '%.1f')])
%     text(xend*0.9, yend*1.05, ['Tend = ', num2str(tend, '%.1f')])
    text(xend*0.98, yend*1.03, ['s_{end} = ', num2str(xend, '%.1f')], 'Interpreter', 'tex')
    text(xend*0.98, yend*1.08, ['t_{end} = ', num2str(tend, '%.1f')], 'Interpreter', 'tex')
    text(xend*0.98, yend*(1.08+0.05), ['v_{end} = ', num2str(vend, '%.1f')], 'Interpreter', 'tex')

    hold on;
end
text(3, 0.4, ['v_{0} = ', num2str(v0, '%.1f')], 'Interpreter', 'tex')

xlim([0, 85]) % 设置 x 轴的范围为 0 到 100
ylim([0, 3.7]) % 设置 x 轴的范围为 0 到 100

xlabel('s(m)') % 设置 x 轴标签
ylabel('l(m)') % 设置 y 轴标签

validList
%% 碰撞检验
