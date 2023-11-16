function [actionTillState,optimalAction]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,CalibrationVars)
%% 打印
if CalibrationVars.debugFlag==1
    fprintf('CalibrationVars.numOfMaxSteps = %d\n',CalibrationVars.numOfMaxSteps);
    fprintf('CalibrationVars.accel = %f\n',CalibrationVars.accel);
    fprintf('CalibrationVars.decel = %f\n',CalibrationVars.decel);
    fprintf('CalibrationVars.stepLength = %f\n',CalibrationVars.stepLength);
    fprintf('CalibrationVars.gamma = %f\n',CalibrationVars.gamma);
    fprintf('CalibrationVars.numOfIteration = %d\n',CalibrationVars.numOfIteration);
    %     fprintf('CalibrationVars.a_min = %f\n',CalibrationVars.a_min);
    %     fprintf('CalibrationVars.a_max = %f\n',CalibrationVars.a_max);
    %     fprintf('CalibrationVars.tou = %f\n',CalibrationVars.tou);
    fprintf('CalibrationVars.a_lateral = %f\n',CalibrationVars.a_lateral);
    fprintf('CalibrationVars.v_min_decel = %f\n',CalibrationVars.v_min_decel);
    fprintf('CalibrationVars.p_actionTime = %f\n',CalibrationVars.p_actionTime);
    fprintf('CalibrationVars.d_safe = %f\n',CalibrationVars.d_safe);
    fprintf('CalibrationVars.d_max = %f\n',CalibrationVars.d_max);
    fprintf('CalibrationVars.t_re = %f\n',CalibrationVars.t_re);
    %     fprintf('CalibrationVars.w_lane = %f\n',CalibrationVars.w_lane);
    %     fprintf('CalibrationVars.w_veh = %f\n',CalibrationVars.w_veh);
    fprintf('CalibrationVars.epsilonSwitch = %d\n',CalibrationVars.epsilonSwitch);
    %     fprintf('CalibrationVars.multipleOfLaneChange = %f\n',CalibrationVars.multipleOfLaneChange);
    %     fprintf('CalibrationVars.multipleOfNearGoalStateLane = %f\n',CalibrationVars.multipleOfNearGoalStateLane);
    %     fprintf('CalibrationVars.multipleOfNearGoalStateSpeed = %f\n',CalibrationVars.multipleOfNearGoalStateSpeed);
    %     fprintf('CalibrationVars.multipleOfKeepGoalState = %f\n',CalibrationVars.multipleOfKeepGoalState);
    fprintf('CalibrationVars.debugFlag = %d\n',int16(CalibrationVars.debugFlag));
    fprintf('CalibrationVars.w1 = %f\n',CalibrationVars.w1);
    fprintf('CalibrationVars.w2 = %f\n',CalibrationVars.w2);

end
%% 用于将v_maxSequcence中的每个元素与v_maxVehicle的最小值进行比较并赋值

% v_maxVehicle = 10; % 替换为您的v_maxVehicle值
% v_maxSequcence = [5, 8, 12, 6, 10]; % 替换为您的v_maxSequcence数组
for i = 1:length(v_maxSequcence)
    v_maxSequcence(i) = min(v_maxSequcence(i), v_maxVehicle);
end
% disp(v_maxSequcence);
%% 计算当前和未来在st图上的障碍物
obstacleMapFuture=obstacleMap(2:51);
obstacleMapCurrent=obstacleMap(1);
%% 标定量
numOfMaxSteps=CalibrationVars.numOfMaxSteps; %=50;
accel=CalibrationVars.accel;%=2.5;
accelComfort=accel/2;
decel=CalibrationVars.decel;%=-4;
decelComfort=accel/2;
stepLength=CalibrationVars.stepLength;%=0.5; % s
gamma=CalibrationVars.gamma;%=0.9; % TBD
numOfIteration=CalibrationVars.numOfIteration;%=1000;
d_max=CalibrationVars.d_max; %=60
%% 初始化出参
actionTillState=zeros(numOfMaxSteps,1);
optimalAction=actionTillState;
%% 赋值state
state.speed = v_0;
state.s = s_0;
state.minDistance = d_max;
if ~isempty(obstacleMapCurrent{1})
    % 定义不可用s坐标区间
    intervals = obstacleMapCurrent{1}(:,1:2);
    % 要检查的值
    value = s_0;
    % 计算到每个区间的距离
    minDistance = d_max;
    for i = 1:size(intervals, 1)
        if value < intervals(i, 1)
            distance = intervals(i, 1) - value;
        elseif value > intervals(i, 2)
            distance = value - intervals(i, 2);
        else
            % 如果该值在区间内，距离为0
            distance = 0;
        end
        % 保存最小距离
        if distance < minDistance
            minDistance = distance;
        end
    end
    % 判断值是否在不可用区间内
    if minDistance == 0
        return;
    else
        state.minDistance=minDistance;
    end
end
%% 初始化node
node=treenode(state,actionTillState,numOfIteration);
%赋值根节点
node(1).state=state;
numOfnodes=1;%节点数量
% coder.varsize('node',[1,numOfIteration]);
% coder.varsize('node.children');
% coder.varsize('node.beBorn');
%初始化
% ICVtraj.trajPoint=zeros(4,3); % laneindex—s—t
% ICVtrajs=repmat(ICVtraj,[1,numOfVehicles]);
profit=-100000;
%% 搜索过程
for i=1:numOfIteration
    doSimulation=0;
    safeFlag=0;%代码生成
    nodeSimulation=node(1);%代码生成
    n=1;%根节点开始
    while (1)
        while node(n).isFullyExpanded==1 && node(n).Generation~=numOfMaxSteps && nnz(node(n).children)~=0%扩展完 不是终节点 有子节点
            %UCT叶节点
            n=selection(node,n,CalibrationVars);
        end
        if node(n).Generation==numOfMaxSteps%当前节点为终节点
            break
        end
        if node(n).isFullyExpanded~=1
            [nodeIndexNew,node,numOfnodes]=expension(node,n,stepLength,accel,decel,accelComfort,decelComfort,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,obstacleMapFuture,numOfnodes,CalibrationVars);
            if nodeIndexNew~=0%生成新子节点
                n=nodeIndexNew;
                doSimulation=1;
                break
            end
        end
        if nnz(node(n).children)==0%无子节点
            break
        end
    end
    r=zeros(1,numOfMaxSteps-node(n).Generation)-1;%当前节点到终节点奖励，当前节点为终节点时r为空
    if doSimulation==1
        nodeSimulation=node(n);
        for j=1:numOfMaxSteps-node(n).Generation
            %             [nSimulation,nodeSimulation]=expension(nodeSimulation,nSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max);
            if j==1
                %                 [safeFlag,nodeSimulation,beBorn]=expensionSimulation(nodeSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max,ICVID,ICVtrajs,vehicleGoalState,...,
                %                     failVehicleIdList,numOfFailVehicle,CalibrationVars,statePreDecision);
                [safeFlag,nodeSimulation,beBorn]=expensionSimulation(nodeSimulation,stepLength,accel,decel,accelComfort,decelComfort,...,
                    sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,obstacleMapFuture,CalibrationVars);
                node(n).beBorn=beBorn;
                if nnz(beBorn)==0
                    node(n).isFullyExpanded=1;
                end
            else
                %                 [safeFlag,nodeSimulation,~]=expensionSimulation(nodeSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max,ICVID,ICVtrajs,vehicleGoalState,...,
                %                     failVehicleIdList,numOfFailVehicle,CalibrationVars,statePreDecision);
                [safeFlag,nodeSimulation,~]=expensionSimulation(nodeSimulation,stepLength,accel,decel,accelComfort,decelComfort,...,
                    sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,obstacleMapFuture,CalibrationVars);
            end
            %             if isequal([nodeSimulation.state.laneIndex]',vehicleGoalState(:,2)) && j==numOfMaxSteps-node(n).Generation
            %                 return;
            %             end
            if safeFlag==0%无子节点
                break
            else
                %                 r(k)=reward(nodeSimulation(nodeSimulation(nSimulation).parent).state,nodeSimulation(nSimulation).state,numOfVehicles);
                r(j)=nodeSimulation.actionReward;
            end
        end
    end
    %累积回报
    gainsSimulation=0;
    for r_index=1:length(r)
        gainsSimulation=gainsSimulation+(gamma.^(r_index-1))*r(r_index);
    end
    %do backpropagation----------------------------------------------------
    [node,rewardFormRootNode]=backpropagation(n,gainsSimulation,node,gamma);
    %----------------------------------------------------------------------
    %最优路径
    if profit<rewardFormRootNode
        if node(n).Generation==numOfMaxSteps
            profit=rewardFormRootNode;
            optimalAction=node(n).actionTillState;
        elseif doSimulation==1
            if safeFlag~=0 && nodeSimulation.Generation==numOfMaxSteps
                profit=rewardFormRootNode;
                optimalAction=nodeSimulation.actionTillState;
            end
        end
    end
end
%决策输出
nStar=selectionBest(node);
actionTillState=node(nStar).actionTillState;
% % 将输出改为char类型
% actionTillStateString=action2actionChar(actionTillState');
% optimalActionString=action2actionChar(optimalAction');

if CalibrationVars.debugFlag==1
    % fprintf('actionTillState = %d\n',actionTillState);
    % fprintf('optimalAction = %d\n',optimalAction);
    fprintf('actionTillState= \t');
    for i=1:length(actionTillState)
        if i==length(actionTillState)
            fprintf('%d\n',actionTillState(i));
        else
            fprintf('%d\t',actionTillState(i));
        end
    end
    fprintf('optimalAction= \t');
    for i=1:length(optimalAction)
        if i==length(optimalAction)
            fprintf('%d\n',optimalAction(i));
        else
            fprintf('%d\t',optimalAction(i));
        end
    end
end
end
% function [C]=action2actionChar(A)
% % Example data
% % Convert matrix to char array
% [m, n] = size(A);
% B = char(zeros(1, m*n*2-1)); % Preallocate char array
% index = 1;
% for i = 1:m
%     for j = 1:n
%         numStr = sprintf('%d',int16(A(i,j)));
%         B(index:index+length(numStr)-1) = numStr;
%         index = index + length(numStr);
%         % Add comma (except for last column)
%         if j < n
%             B(index) = ',';
%             index = index + 1;
%         end
%     end
%     % Add semicolon (except for last row)
%     if i < m
%         B(index) = ';';
%         index = index + 1;
%     end
% end
% C = repmat(' ', 1, 50);
% if length(B)<50
%     % 确定输入字符数组的长度
%     input_length = length(B);
%     % 先将输出字符数组的前部分与输入字符数组等同
%     C(1:input_length) = B;
%     % 使用循环填充输出字符数组的后半部分
%     for i = (input_length+1):50
%         C(i) = ' ';
%     end
% elseif length(B)>50
%     C=B(1:50);
% end
% end

% function linkMap=initializationOfLinkMap(numOfLanes)
% laneMap.vehicleId=zeros(10,1); %laneMap
% laneMap.s=zeros(10,1)+10^6;
% laneMap.speed=zeros(10,1);
% linkMap = repmat(laneMap,1,numOfLanes);
% end
% function [state,actionTillState]=initializationOfstate(numOfVehicles,numOfMaxSteps)
% stateOfVehcile.vehicleId=0;
% stateOfVehcile.speed = 0;
% stateOfVehcile.laneIndex = 0;
% % stateOfVehcile.targetspeed = 0;
% % stateOfVehcile.targetLaneIndex = 0;
% stateOfVehcile.vehIndex = 0;
% state = repmat(stateOfVehcile,1,numOfVehicles);
% actionTillState=zeros(numOfMaxSteps,1);
% end
function node=treenode(state,actionTillState,numOfIteration)
self.state=state;
self.beBorn=1:1:5;
self.isFullyExpanded=0;
self.parent=0;
self.numVisits=0;
self.totalReward=0;
self.actionReward=0;
self.children=zeros(1,5);
self.Generation=0;
self.actionTillState=actionTillState;
node=repmat(self,1,numOfIteration+1);
% ？是否加上state.minDIS 从当前时刻的obastaclemap（新增的入参）得出？
end

% % 要检查的值
% value = sList(iterInSList);
% % 计算到每个区间的距离
% minDistance = Inf;
% for i = 1:size(intervals, 1)
%     if value < intervals(i, 1)
%         distance = intervals(i, 1) - value;
%     elseif value > intervals(i, 2)
%         distance = value - intervals(i, 2);
%     else
%         % 如果该值在区间内，距离为0
%         distance = 0;
%     end
%     % 保存最小距离
%     if distance < minDistance
%         minDistance = distance;
%     end
% end
% % 判断值是否在不可用区间内
% if minDistance == 0
%     safeFlag=0;
%     break;
% else
%     if minDistanceInSList<minDistance
%         minDistanceInSList=minDistance;
%     end
% end

% % 创建一个cell数组
% myCell = cell(1, 50);
%
% % 在第一个元素中添加2x3的矩阵,表示0.1秒时刻的不可行驶区间和车速
% myCell{1} = ones(2, 3);
%
% % 在第二个元素中添加5x3的矩阵,表示0.2秒时刻的不可行驶区间和车速
% myCell{2} = ones(5, 3);

% % 定义不可用s坐标区间
% intervals = [2, 3; 5, 6];
%
% % 要检查的值
% value = 2;
%
% % 计算到每个区间的距离
% minDistance = Inf;
%
% for i = 1:size(intervals, 1)
%     if value < intervals(i, 1)
%         distance = intervals(i, 1) - value;
%     elseif value > intervals(i, 2)
%         distance = value - intervals(i, 2);
%     else
%         % 如果该值在区间内，距离为0
%         distance = 0;
%     end
%
%     % 保存最小距离
%     if distance < minDistance
%         minDistance = distance;
%     end
% end
%
% % 判断值是否在不可用区间内
% if minDistance == 0
%     fprintf('%d 在不可用区间内\n', value);
% else
%     fprintf('%d 不在不可用区间内，最近的距离为 %d\n', value, minDistance);
% end
