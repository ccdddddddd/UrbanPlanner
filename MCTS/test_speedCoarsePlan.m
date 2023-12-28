clc
clear
close all
%% 标定量
CalibrationVars.numOfMaxSteps=10;
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
CalibrationVars.stepLength=0.5; % s
CalibrationVars.gamma=0.8; % TBD
CalibrationVars.numOfIteration=5000;
CalibrationVars.a_lateral=3;
CalibrationVars.v_min_decel=0;
CalibrationVars.p_actionTime=0.5;

CalibrationVars.d_safe=1;
CalibrationVars.d_max=60;

CalibrationVars.t_re=0.5;
CalibrationVars.epsilonSwitch=int16(5);
CalibrationVars.UCBswitch=1;
CalibrationVars.UCBconstant=1;
CalibrationVars.debugFlag=false;
CalibrationVars.w1=0.7; % 距离权重
CalibrationVars.w2=1-CalibrationVars.w1; % 速度权重
%% 选择用例并构造输入
obstacleMap = cell(1, 51);
rectangles={};
testCase=1;
switch testCase
    case 1
        v_0=10;
        s_0=0;
        v_maxVehicle=30;
        sSequcence=[0,100,200];
        rSequcence=[2000,2000,2000];
        v_maxSequcence=[20,20,20];
        % 创建一个cell数组obstacleMap，并构造入参
        obstacleMapOrigin={[1,1;2,9],[3,23;4,29],[2,40;5,45]};
        for iterInObstacleMapOrigin=1:1:length(obstacleMapOrigin)
            % 已知长方形的两个对角顶点
            point1 = obstacleMapOrigin{iterInObstacleMapOrigin}(1,:); %[1, 4];
            point2 = obstacleMapOrigin{iterInObstacleMapOrigin}(2,:);% [2, 5];
            % 计算长方形的四个顶点
            x1 = min(point1(1), point2(1));
            x2 = max(point1(1), point2(1));
            y1 = min(point1(2), point2(2));
            y2 = max(point1(2), point2(2));
            vertices = [x1, y1; x1, y2; x2, y2; x2, y1];
            rectangles{end+1}=vertices;
        end
        for iterInMap=1:1:51
            % 定义t=tx直线的横坐标
            tx = (iterInMap-1)*0.1;
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
            if ~isempty(crossPoints)
                obstacleMap{iterInMap}=crossPoints;
            end
        end
    case 2
        obstacleMap = cell(1, 51);
        v_0=20;
        s_0=0;
        v_maxVehicle=30;
        sSequcence=[0,30,200];
        rSequcence=[200,50,50];
        v_maxSequcence=[20,20,20];
    case 3
        obstacleMap = cell(1, 51);
        v_0=15;
        s_0=0;
        v_maxVehicle=30;
        sSequcence=[0,30,50,200];
        rSequcence=[200,200,200,200];
        v_maxSequcence=[20,5,5,20];
end
%% case
[actionTillState,optimalAction]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,CalibrationVars);
optimalAction
actionTillState

% 创建一个新的图形窗口
figure('Position', [0, 100, 900, 100+400]) % 创建一个指定大小的图形窗口

%% 画障碍物
% 遍历每个长方形
for i = 1:length(rectangles)
    rect = rectangles{i};
    % 添加第一个顶点到最后一个顶点，形成一个闭合图形
    rect = [rect; rect(1,:)];
    % 绘制长方形
    plot(rect(:,1), rect(:,2), '-o');
    hold on;
end
%% 画位移时间图
accel=CalibrationVars.accel;%=2.5;
accelComfort=accel/2;
decel=CalibrationVars.decel;%=-4;
decelComfort=accel/2;

speed = v_0; % 当前车速
s = s_0; % 当前位置
% v_max = v_maxVehicle; % 最高车速
% 计算每一帧的车速和位置
dt = 0.1; % 每一帧的时间间隔
frames = 50; % 总共计算的帧数
speedList=zeros(frames,1);
sList=zeros(frames,1);
timeList=0.1:0.1:5;
for i = 1:1:frames
    % 判断当前状态（加速、匀速、减速、静止）
    actionNum=ceil(i/5);
    action=optimalAction(actionNum);
    a=0;
    switch action
        case 1
            a=decel;
        case 2
            a=decelComfort;
        case 3
            a=0;
        case 4
            a=accelComfort;
        case 5
            a=accel;
    end
    speedNew = median([speed + a * dt,0,v_maxVehicle]);
    s = s + (speed+speedNew)/2 * dt;
    % 输出当前帧的车速和位置
    speedList(i)=speedNew;
    sList(i)=s;
    speed=speedNew;
end
plot([0,timeList],[s_0;sList]);
% 标注每一段的加速度
accelerations=zeros(1,CalibrationVars.numOfMaxSteps);
for i=1:1:CalibrationVars.numOfMaxSteps
    a=0;
    switch optimalAction(i)
        case 1
            a=decel;
        case 2
            a=decelComfort;
        case 3
            a=0;
        case 4
            a=accelComfort;
        case 5
            a=accel;
    end

    accelerations(i)=a;
end
for i = 1:length(accelerations)
    segment_start_time = (i-1) * CalibrationVars.stepLength;
    segment_end_time = i * CalibrationVars.stepLength;
    segment_mid_time = (segment_start_time + segment_end_time) / 2;
    segment_acceleration = accelerations(i);
    text(segment_mid_time, sList(5*i)+1, sprintf('a= %.2f', segment_acceleration), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(segment_mid_time, sList(5*i)+3, sprintf('v= %.2f', speedList(5*i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
% 每隔5个点标红
hold on;
for i = 5:5:length(sList)
    plot(timeList(i), sList(i), 'ro');
end

%% 其他绘图
% 设置坐标轴范围
%     xlim([min(rectangles{1}(:,1))-1, max(rectangles{1}(:,1))+1]);
%     ylim([min(rectangles{1}(:,2))-1, max(rectangles{1}(:,2))+1]);
% 添加标题和坐标轴标签
title('Displacement-Time Curve of MCTS Segmented Motion');
xlabel('t (s)');
ylabel('s (m)');
switch testCase
    case 1
xlim([0, 5]) % 设置 x 轴的范围为 0 到 100
ylim([0, 50]) % 设置 x 轴的范围为 0 到 100
    case 2
xlim([0, 5]) % 设置 x 轴的范围为 0 到 100
ylim([0, 75]) % 设置 x 轴的范围为 0 到 100
            case 3
xlim([0, 5]) % 设置 x 轴的范围为 0 到 100
ylim([0, 50]) % 设置 x 轴的范围为 0 到 100

end

% 添加网格线
grid on;
% 添加图例
%     legend('Rectangle 1', 'Rectangle 2', 'Rectangle 3');
% % 定义长方形的四个顶点坐标
% rect1 = [1, 1; 1, 2; 2, 2; 2, 1];
% rect2 = [3, 3; 3, 4; 4, 4; 4, 3];
% rect3 = [1, 4; 1, 5; 2, 5; 2, 4];
% % 其他长方形定义...
% rectangles = {rect1, rect2, rect3}; % 将所有长方形放入一个cell数组中，可以根据实际情况修改
% plot([0,timeList],[v_0;speedList]);

