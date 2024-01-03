clc
clear
close all
%% 标定量
CalibrationVars.numOfMaxSteps=10;
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
CalibrationVars.stepLength=0.5; % s
CalibrationVars.gamma=0.8; % TBD
CalibrationVars.numOfIteration=4000;
CalibrationVars.a_lateral=3;
CalibrationVars.v_min_decel=0;
CalibrationVars.p_actionTime=0.5;
CalibrationVars.d_safe=1;
CalibrationVars.d_max=60;
CalibrationVars.t_re=0.5;
CalibrationVars.UCBswitch=0;
CalibrationVars.UCBconstant=1;
CalibrationVars.nodeRef=1; % 1：node(1).numVisits  2：node(predecessor).numVisits
CalibrationVars.epsilonSwitch=int16(1);
CalibrationVars.debugFlag=false;
CalibrationVars.w1=0.7; % 距离权重
CalibrationVars.w2=1-CalibrationVars.w1; % 速度权重
%% 选择用例并构造输入
obstacleMap = cell(1, 51);
rectangles={};
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
%% case
% Initialize arrays to store the combinations and their corresponding profits
combinations = [];
profits = [];
probOfSuccess = [];
timeCosts = [];

% Define the range for UCBconstant
UCBconstant_range = [0.1:0.1:2, 3:1:20]; % [0.1,0.2]; % 
runtimeMax=100;
% Loop through all combinations of UCBswitch, epsilonSwitch, nodeRef, and UCBconstant
for UCBswitch = 1
    for epsilonSwitch = 1
        for nodeRef = 1:2
            for UCBconstant = UCBconstant_range
                % Calculate the profit for the current combination
                CalibrationVars.UCBswitch=UCBswitch;
                CalibrationVars.UCBconstant=UCBconstant;
                CalibrationVars.nodeRef=nodeRef; % 1：node(1).numVisits  2：node(predecessor).numVisits
                CalibrationVars.epsilonSwitch=epsilonSwitch;
                profitSum=0;
                SuccessSum=0;
                tic;
                for runtime=1:1:runtimeMax
                    [actionTillState,optimalAction,profit]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,CalibrationVars);
                    if ~any(optimalAction == 0)
                        SuccessSum=SuccessSum+1;
                        profitSum=profitSum+profit;
                    end
                    % profitSum=profitSum+max(-10,profit);
                end
                timeCost=toc/runtimeMax;
                profitMean=profitSum/SuccessSum;
                probOfSuccessOnce=SuccessSum/runtimeMax;
                % Store the combination and profit in the arrays
                combinations = [combinations; UCBswitch, epsilonSwitch, nodeRef, UCBconstant];
                profits = [profits; profitMean];
                probOfSuccess = [probOfSuccess; probOfSuccessOnce];
                timeCosts=[timeCosts;timeCost];
            end
        end
    end
end
for UCBswitch = 0
    for epsilonSwitch = 1:1:3
        for nodeRef = 1:2
            for UCBconstant = 0
                % Calculate the profit for the current combination
                CalibrationVars.UCBswitch=UCBswitch;
                CalibrationVars.UCBconstant=UCBconstant;
                CalibrationVars.nodeRef=nodeRef; % 1：node(1).numVisits  2：node(predecessor).numVisits
                CalibrationVars.epsilonSwitch=epsilonSwitch;
                profitSum=0;
                SuccessSum=0;
                tic;
                for runtime=1:1:runtimeMax
                    [actionTillState,optimalAction,profit]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,CalibrationVars);
                    if ~any(optimalAction == 0)
                        SuccessSum=SuccessSum+1;
                        profitSum=profitSum+profit;
                    end
                    % profitSum=profitSum+max(-10,profit);
                end
                timeCost=toc/runtimeMax;
                profitMean=profitSum/SuccessSum;
                probOfSuccessOnce=SuccessSum/runtimeMax;
                % Store the combination and profit in the arrays
                combinations = [combinations; UCBswitch, epsilonSwitch, nodeRef, UCBconstant];
                profits = [profits; profitMean];
                probOfSuccess = [probOfSuccess; probOfSuccessOnce];
                timeCosts=[timeCosts;timeCost];
            end
        end
    end
end
for UCBswitch = 0
    for epsilonSwitch = 0.1:0.1:0.9
        for nodeRef = 1
            for UCBconstant = 0
                % Calculate the profit for the current combination
                CalibrationVars.UCBswitch=UCBswitch;
                CalibrationVars.UCBconstant=UCBconstant;
                CalibrationVars.nodeRef=nodeRef; % 1：node(1).numVisits  2：node(predecessor).numVisits
                CalibrationVars.epsilonSwitch=epsilonSwitch;
                profitSum=0;
                SuccessSum=0;
                tic;
                for runtime=1:1:runtimeMax
                    [actionTillState,optimalAction,profit]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,CalibrationVars);
                    if ~any(optimalAction == 0)
                        SuccessSum=SuccessSum+1;
                        profitSum=profitSum+profit;
                    end
                    % profitSum=profitSum+max(-10,profit);
                end
                timeCost=toc/runtimeMax;
                profitMean=profitSum/SuccessSum;
                probOfSuccessOnce=SuccessSum/runtimeMax;
                % Store the combination and profit in the arrays
                combinations = [combinations; UCBswitch, epsilonSwitch, nodeRef, UCBconstant];
                profits = [profits; profitMean];
                probOfSuccess = [probOfSuccess; probOfSuccessOnce];
                timeCosts=[timeCosts;timeCost];
            end
        end
    end
end

% Combine the combinations and profits into a single array
result_array = [combinations, profits,probOfSuccess,timeCosts];

% Display the result array
disp(result_array);
save('calibrationTest20240103.mat', 'result_array');

