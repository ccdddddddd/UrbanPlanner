function [actionTillStateString,optimalActionString]=CollaborativeLaneChange(linkMapIds,linkMapSpeeds,linkMapSs,linkMapLaneIndexs,stateIds,stateSpeeds,stateLanes,stateTargetSpeeds,...
    stateTargetLanes,statePreDecision,failVehicleIdList,v_max,numOfVehicles,numOfLanes,CalibrationVars)
if CalibrationVars.debugFlag==1
    fprintf('CalibrationVars.numOfMaxSteps = %d\n',CalibrationVars.numOfMaxSteps);
    fprintf('CalibrationVars.accel = %f\n',CalibrationVars.accel);
    fprintf('CalibrationVars.decel = %f\n',CalibrationVars.decel);
    fprintf('CalibrationVars.stepLength = %f\n',CalibrationVars.stepLength);
    fprintf('CalibrationVars.gamma = %f\n',CalibrationVars.gamma);
    fprintf('CalibrationVars.numOfIteration = %d\n',CalibrationVars.numOfIteration);
    fprintf('CalibrationVars.a_min = %f\n',CalibrationVars.a_min);
    fprintf('CalibrationVars.a_max = %f\n',CalibrationVars.a_max);
    fprintf('CalibrationVars.tou = %f\n',CalibrationVars.tou);
    fprintf('CalibrationVars.v_min_decel = %f\n',CalibrationVars.v_min_decel);
    fprintf('CalibrationVars.p_actionTime = %f\n',CalibrationVars.p_actionTime);
    fprintf('CalibrationVars.d_safe = %f\n',CalibrationVars.d_safe);
    fprintf('CalibrationVars.t_re = %f\n',CalibrationVars.t_re);
    fprintf('CalibrationVars.w_lane = %f\n',CalibrationVars.w_lane);
    fprintf('CalibrationVars.w_veh = %f\n',CalibrationVars.w_veh);
    fprintf('CalibrationVars.epsilonSwitch = %d\n',CalibrationVars.epsilonSwitch);
    fprintf('CalibrationVars.multipleOfLaneChange = %f\n',CalibrationVars.multipleOfLaneChange);
    fprintf('CalibrationVars.multipleOfNearGoalStateLane = %f\n',CalibrationVars.multipleOfNearGoalStateLane);
    fprintf('CalibrationVars.multipleOfNearGoalStateSpeed = %f\n',CalibrationVars.multipleOfNearGoalStateSpeed);
    fprintf('CalibrationVars.multipleOfKeepGoalState = %f\n',CalibrationVars.multipleOfKeepGoalState);
    fprintf('CalibrationVars.debugFlag = %d\n',int16(CalibrationVars.debugFlag));
    fprintf('failVehicleIdList= \t');
    for i=1:length(failVehicleIdList)
        if i==length(failVehicleIdList)
            fprintf('%d\n',failVehicleIdList(i));
        else
            fprintf('%d\t',failVehicleIdList(i));
        end
    end
    fprintf('linkMapIds= \t');
    for i=1:length(linkMapIds)
        if i==length(linkMapIds)
            fprintf('%d\n',linkMapIds(i));
        else
            fprintf('%d\t',linkMapIds(i));
        end
    end
    fprintf('linkMapLaneIndexs= \t');
    for i=1:length(linkMapLaneIndexs)
        if i==length(linkMapLaneIndexs)
            fprintf('%d\n',linkMapLaneIndexs(i));
        else
            fprintf('%d\t',linkMapLaneIndexs(i));
        end
    end
    fprintf('linkMapSpeeds = \t');
    for i=1:length(linkMapSpeeds)
        if i==length(linkMapSpeeds)
            fprintf('%f\n',linkMapSpeeds(i));
        else
            fprintf('%f\t',linkMapSpeeds(i));
        end
    end
    fprintf('linkMapSs = \t');
    for i=1:length(linkMapSs)
        if i==length(linkMapSs)
            fprintf('%f\n',linkMapSs(i));
        else
            fprintf('%f\t',linkMapSs(i));
        end
    end
    fprintf('numOfLanes = %d\n',numOfLanes);
    fprintf('numOfVehicles = %d\n',numOfVehicles);
    fprintf('stateIds= \t');
    for i=1:length(stateIds)
        if i==length(stateIds)
            fprintf('%d\n',stateIds(i));
        else
            fprintf('%d\t',stateIds(i));
        end
    end
    fprintf('stateLanes= \t');
    for i=1:length(stateLanes)
        if i==length(stateLanes)
            fprintf('%d\n',stateLanes(i));
        else
            fprintf('%d\t',stateLanes(i));
        end
    end
    fprintf('stateSpeeds = \t');
    for i=1:length(stateSpeeds)
        if i==length(stateSpeeds)
            fprintf('%f\n',stateSpeeds(i));
        else
            fprintf('%f\t',stateSpeeds(i));
        end
    end
    fprintf('stateTargetLanes= \t');
    for i=1:length(stateTargetLanes)
        if i==length(stateTargetLanes)
            fprintf('%d\n',stateTargetLanes(i));
        else
            fprintf('%d\t',stateTargetLanes(i));
        end
    end
    fprintf('stateTargetSpeeds = \t');
    for i=1:length(stateTargetSpeeds)
        if i==length(stateTargetSpeeds)
            fprintf('%f\n',stateTargetSpeeds(i));
        else
            fprintf('%f\t',stateTargetSpeeds(i));
        end
    end
    fprintf('v_max = %f\n',v_max);
    
end
%int16转double
numOfVehicles=double(numOfVehicles);
linkMapIds=double(linkMapIds);
stateIds=double(stateIds);
stateLanes=double(stateLanes);
stateTargetLanes=double(stateTargetLanes);
failVehicleIdList=double(failVehicleIdList);
%标定量
numOfMaxSteps=CalibrationVars.numOfMaxSteps; %=5;
accel=CalibrationVars.accel;%=1.5;
decel=CalibrationVars.decel;%=-1.5;
stepLength=CalibrationVars.stepLength;%=2; % s
gamma=CalibrationVars.gamma;%=0.9; % TBD
numOfIteration=CalibrationVars.numOfIteration;%=2500;
%初始化state linkMap node
[state,actionTillState]=initializationOfstate(numOfVehicles,numOfMaxSteps);
linkMap=initializationOfLinkMap(numOfLanes);
node=treenode(state,actionTillState,linkMap,numOfVehicles,numOfIteration);
%赋值state linkMap
for i=1:numOfLanes
    index=(linkMapLaneIndexs==i & linkMapIds~=0);
    linkMap(i).vehicleId(1:length(linkMapIds(index)))=linkMapIds(index);
    linkMap(i).s(1:length(linkMapIds(index)))=linkMapSs(index);
    linkMap(i).speed(1:length(linkMapIds(index)))=linkMapSpeeds(index);
end
% linkmap的排序
for iterLane2sort=1:1:numOfLanes
    [linkMap(iterLane2sort).s,I]=sort(linkMap(iterLane2sort).s);%该车道车辆位置从小到大排序
    linkMap(iterLane2sort).vehicleId=linkMap(iterLane2sort).vehicleId(I);%更新车辆ID顺序
    linkMap(iterLane2sort).speed=linkMap(iterLane2sort).speed(I);%更新车辆速度顺序
end
ICVID=zeros(1,numOfVehicles);%ICVID网联车Id组成的数组
vehicleGoalState=zeros(numOfVehicles,2);%网联车目标状态
for vehIndex=1:numOfVehicles
    ICVID(vehIndex)=stateIds(vehIndex);
    state(vehIndex).vehicleId = stateIds(vehIndex);
    state(vehIndex).speed = stateSpeeds(vehIndex);
    state(vehIndex).laneIndex = stateLanes(vehIndex);
%     state(vehIndex).targetspeed = stateTargetSpeeds(vehIndex);
%     state(vehIndex).targetLaneIndex = stateTargetLanes(vehIndex);
    state_vehIndex=find((linkMap(stateLanes(vehIndex)).vehicleId)==state(vehIndex).vehicleId,1); 
    state(vehIndex).vehIndex=state_vehIndex(1);
    vehicleGoalState(vehIndex,1)=stateTargetSpeeds(vehIndex);
    vehicleGoalState(vehIndex,2)=stateTargetLanes(vehIndex);
end
%赋值根节点
node(1).state=state;
node(1).linkMap=linkMap;
numOfnodes=1;%节点数量
% coder.varsize('node',[1,numOfIteration]);
% coder.varsize('node.children');
% coder.varsize('node.beBorn');
%初始化
% ICVtraj.trajPoint=zeros(4,3); % laneindex—s—t
% ICVtrajs=repmat(ICVtraj,[1,numOfVehicles]);
ICVtrajs=zeros(numOfVehicles,4,3);%i（网联车序号） j(时刻) k (laneindex—s—t)
profit=-100000;
optimalAction=actionTillState;
numOfFailVehicle=nnz(failVehicleIdList);
for i=1:numOfIteration
    doSimulation=0;
    safeFlag=0;%代码生成
    nodeSimulation=node(1);%代码生成
    n=1;%根节点开始
    while (1)
        while node(n).isFullyExpanded==1&&node(n).Generation~=numOfMaxSteps&&nnz(node(n).children)~=0%扩展完 不是终节点 有子节点
            %UCT叶节点
            n=selection(node,n,CalibrationVars);
        end
        if node(n).Generation==numOfMaxSteps%当前节点为终节点
            break
        end
        if node(n).isFullyExpanded~=1
            [nodeIndexNew,node,numOfnodes]=expension(node,n,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max,ICVID,numOfnodes,ICVtrajs,vehicleGoalState,...,
                failVehicleIdList,numOfFailVehicle,CalibrationVars,statePreDecision);
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
    r=zeros(1,numOfMaxSteps-node(n).Generation)-50;%当前节点到终节点奖励，当前节点为终节点时r为空
    if doSimulation==1
        nodeSimulation=node(n);
        for j=1:numOfMaxSteps-node(n).Generation
%             [nSimulation,nodeSimulation]=expension(nodeSimulation,nSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max);
            if j==1
                [safeFlag,nodeSimulation,beBorn]=expensionSimulation(nodeSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max,ICVID,ICVtrajs,vehicleGoalState,...,
                    failVehicleIdList,numOfFailVehicle,CalibrationVars,statePreDecision);
                node(n).beBorn=beBorn;
                if nnz(beBorn)==0
                   node(n).isFullyExpanded=1;
                end
            else
                [safeFlag,nodeSimulation,~]=expensionSimulation(nodeSimulation,stepLength,accel,decel,numOfVehicles,numOfLanes,v_max,ICVID,ICVtrajs,vehicleGoalState,...,
                    failVehicleIdList,numOfFailVehicle,CalibrationVars,statePreDecision);
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
            if safeFlag~=0&&nodeSimulation.Generation==numOfMaxSteps
            profit=rewardFormRootNode;
            optimalAction=nodeSimulation.actionTillState;
            end
        end  
    end 
end
%决策输出
nStar=selectionBest(node);
actionTillState=node(nStar).actionTillState;
% 将输出改为char类型
actionTillStateString=action2actionChar(actionTillState');
optimalActionString=action2actionChar(optimalAction');

if CalibrationVars.debugFlag==1
    fprintf('actionTillStateString = %s\n',actionTillStateString);
    fprintf('optimalActionString = %s\n',optimalActionString);
end
end
function [C]=action2actionChar(A)
% Example data
% Convert matrix to char array
[m, n] = size(A);
B = char(zeros(1, m*n*2-1)); % Preallocate char array
index = 1;
for i = 1:m
    for j = 1:n
        numStr = sprintf('%d',int16(A(i,j)));
        B(index:index+length(numStr)-1) = numStr;
        index = index + length(numStr);
        % Add comma (except for last column)
        if j < n
            B(index) = ',';
            index = index + 1;
        end
    end
    % Add semicolon (except for last row)
    if i < m
        B(index) = ';';
        index = index + 1;
    end
end
C = repmat(' ', 1, 50);
if length(B)<50
    % 确定输入字符数组的长度
    input_length = length(B);
    % 先将输出字符数组的前部分与输入字符数组等同
    C(1:input_length) = B;
    % 使用循环填充输出字符数组的后半部分
    for i = (input_length+1):50
        C(i) = ' ';
    end
elseif length(B)>50
    C=B(1:50);
end
end

function linkMap=initializationOfLinkMap(numOfLanes)
laneMap.vehicleId=zeros(10,1); %laneMap
laneMap.s=zeros(10,1)+10^6;
laneMap.speed=zeros(10,1);
linkMap = repmat(laneMap,1,numOfLanes);
end
function [state,actionTillState]=initializationOfstate(numOfVehicles,numOfMaxSteps)
stateOfVehcile.vehicleId=0;
stateOfVehcile.speed = 0;
stateOfVehcile.laneIndex = 0;
% stateOfVehcile.targetspeed = 0;
% stateOfVehcile.targetLaneIndex = 0;
stateOfVehcile.vehIndex = 0;
state = repmat(stateOfVehcile,1,numOfVehicles);
actionTillState=zeros(numOfMaxSteps,numOfVehicles);
end
function node=treenode(state,actionTillState,linkMap,numOfVehicles,numOfIteration)
self.state=state;
self.linkMap =linkMap ;
self.beBorn=1:1:5^numOfVehicles;
self.isFullyExpanded=0;
self.parent=0;
self.numVisits=0;
self.totalReward=0;
self.actionReward=0;
self.children=zeros(1,5^numOfVehicles);
self.Generation=0;
self.actionTillState=actionTillState;
node=repmat(self,1,numOfIteration+1);
end