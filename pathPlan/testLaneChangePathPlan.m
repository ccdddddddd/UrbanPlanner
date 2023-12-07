
clear all
close all
% openfig('untitled.fig')
s_0=0;
l_0=0; %1.6
v_0=20;
turningRadius=5;
CalibrationVars.a_lateral=3;
CalibrationVars.wComfort2EfficiencyLaneChange=0.5; % Jchg=w*(aLst/aMax).^2+(1-w)*(xEnd/xEndMax).^2
CalibrationVars.tMaxLaneChange=6;
CalibrationVars.aMaxLaneChange=1.5;
CalibrationVars.aMinLaneChange=-2;
CalibrationVars.vMaxLaneChange=20;
CalibrationVars.vMinLaneChange=6;
CalibrationVars.decel=-4;
CalibrationVars.speedGap=2;
BasicInfo.widthOfVehicle=1.8;
CalibrationVars.d_max=60;
CalibrationVars.d_safe=5;
CalibrationVars.t_re=0.5;
CalibrationVars.MovingRoomFromCenterLane=1;
CalibrationVars.checkTimeGap=0.1;
CalibrationVars.wDis=0.5;
CalibrationVars.wAlat=1-CalibrationVars.wDis;
CalibrationVars.linspaceNum=50;
CalibrationVars.linspaceNumCrossCal=25;
CalibrationVars.linspaceNumALateralCal=10;
offsetTarget2CurrentLane=3.2;
sSequcence=[0,50,51,100];
offsetTarget2CurrentLaneSequcence=[3.2,3.2,3.2,2.4];
headingTargetLaneSequcence=[0,0,-atand((3.2-2.4)/(100-50)),-atand((3.2-2.4)/(100-50))];
headingCurrent=5;
obstacleMap = cell(1, 51);
rectangles={};
% 创建一个cell数组obstacleMap，并构造入参
obstacleMapOrigin={[1,1;2,9],[3,23;4,29]};
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
obstacleMapTargetLane = cell(1, 51);
rectanglesTargetLane={};
% 创建一个cell数组obstacleMap，并构造入参
obstacleMapOrigin={[2,35;5,38]};
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
    rectanglesTargetLane{end+1}=vertices;
end
for iterInMap=1:1:51
    % 定义t=tx直线的横坐标
    tx = (iterInMap-1)*0.1;
    % 初始化交点数组
    crossPoints = [];
    % 遍历每个长方形
    for i = 1:length(rectanglesTargetLane)
        rect = rectanglesTargetLane{i};
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
        obstacleMapTargetLane{iterInMap}=crossPoints;
    end
end
plotFlag=0;
tic
[pathPara,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,sSequcence,offsetTarget2CurrentLaneSequcence...,
    ,headingTargetLaneSequcence,headingCurrent,obstacleMap,obstacleMapTargetLane,CalibrationVars,BasicInfo,plotFlag,rectangles,rectanglesTargetLane) ;
toc