function [nodeIndexNew,node,numOfnodes]=expension(node,n,stepLength,accel,decel,accelComfort,decelComfort,sSequcence,rSequcence,v_maxVehicle,v_maxSequcence,obstacleMapFuture,numOfnodes,CalibrationVars)
% 当前节点创建子节点返回第一个子节点
%% 标定量
p_actionTime=CalibrationVars.p_actionTime; % 0.5
a_lateral=CalibrationVars.a_lateral; % 3
v_min_decel=CalibrationVars.v_min_decel; % 最低车速0
%% 创建子节点
nodeIndexNew=0;%新节点索引
safeFlag=0;
action=0;
state=node(n).state;
stateNew=node(n).state;
%% 遍历各个action，以扩展新节点
while safeFlag==0
    if nnz(node(n).beBorn)==0
        node(n).isFullyExpanded=1;
        break
    end
    beBornList=nonzeros(node(n).beBorn);
    action=beBornList(randi([1,length(beBornList)])); % action 1：急减速、2：缓减速、3：匀速、4：缓加速、5：急加速
    % %         born=node(n).beBorn(randi([1,length(node(n).beBorn)]));%在未出生的孩子中随机出生一个；
    % %         toBeDeleted=find(node(n).beBorn==born);%未出生孩子删掉出生的孩子
    % %         if ~isempty(toBeDeleted)
    % %             node(n).beBorn(toBeDeleted(1))=0;
    % %         end
    node(n).beBorn(action)=0; % 未出生孩子中，删掉出生的孩子
    %         FiveBase=dec2quinary(born-1);
    %         action=zeros(1,numOfVehicles);
    %         action(1,numOfVehicles-length(FiveBase)+1:end)=FiveBase;
    %         action=flip(action+1,2);
    %         laneIndexOfVeh=cat(2,node(n).state.laneIndex);
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

    %% 计算接下来5帧（每一帧0.1秒），从state开始，执行action，每一帧的车速和位置。
    % 已知当前车速speed（speed大于等于0，小于等于最高车速v_max），当前位置s，加速度a。车辆进行匀加速运动，直达减速到零或者加速到最高车速v_max，加速到最高车速v_max之后进行的是匀速运动，减速到0之后静止。
    % 输入参数
    speed = state.speed; % 当前车速
    s = state.s; % 当前位置
    % v_max = v_maxVehicle; % 最高车速
    % 计算每一帧的车速和位置
    dt = 0.1; % 每一帧的时间间隔
    frames = 5; % 总共计算的帧数
    speedList=zeros(frames,1);
    sList=zeros(frames,1);
    for i = 1:1:frames
        % 判断当前状态（加速、匀速、减速、静止）
        speedNew = median([speed + a * dt,0,v_maxVehicle]);
        s = s + (speed+speedNew)/2 * dt;
        % 输出当前帧的车速和位置
        speedList(i)=speedNew;
        sList(i)=s;
        speed=speedNew;
    end
    stateNew.speed = speedList(frames);
    stateNew.s = sList(frames);

    %% 计算曲率与最大车速限制（sSequcence和rSequcence为长度相同的数组，s坐标为sSequcence[i]的时候，对应曲率为rSequcence[i]。s坐标在sSequcence[i]和sSequcence[i+1]之间的时候，通过线性插值得出对应曲率

    %         % 输入sSequcence和rSequcence数组
    %         sSequcence = [1, 2, 3, 4, 5];
    %         rSequcence = [0.1, 0.3, 0.6, 0.8, 0.9];
    % 输入要插值的s坐标
    % s = 2.5;
    % 找到stateNew.s所在的区间
    index = find(sSequcence <= stateNew.s, 1, 'last');
    % 线性插值
    if ~isempty(index) && index < numel(sSequcence)
        s1 = sSequcence(index);
        s2 = sSequcence(index + 1);
        r1 = rSequcence(index);
        r2 = rSequcence(index + 1);
        interpolated_r = r1 + (r2 - r1) * (stateNew.s - s1) / (s2 - s1);
        v1 = v_maxSequcence(index);
        v2 = v_maxSequcence(index + 1);
        v_max = v1 + (v2 - v1) * (stateNew.s - s1) / (s2 - s1);
    else
        fprintf('error:s坐标超出范围\n');
        interpolated_r=100000;
        v_max = v_maxVehicle;
    end
    %         % 输出插值结果
    %         interpolated_r
    actionValidPrereq1= ~( ... % 车速限制
        (state.speed<=v_min_decel && action<3) ||..., % 车速为0的时候不能减速了
        (action==4 && accelComfort*stepLength*p_actionTime+state.speed>v_max) ||..., % 缓缓加速超过限速
        (action==5 && accel*stepLength*p_actionTime+state.speed>v_max) ... % 急加速超过限速
        );
    if actionValidPrereq1
        %         if v_max<10
        %             v_max
        %         end
        actionValidPrereq2=(stateNew.speed).^2/interpolated_r<=a_lateral && stateNew.speed<=v_max*1.1;  % 横向加速度a_lateral限制 && 限速
        if actionValidPrereq2
            %% 碰撞校验
            safeFlag=1;
            minDistanceInSList=CalibrationVars.d_max;
            for iterInSList=1:1:5
                %% 五个帧中每一个帧，进行距离检测
                % 定义不可用s坐标区间
                iInObatacleMap=iterInSList+node(n).Generation*5;
                if ~isempty(obstacleMapFuture{iInObatacleMap})
                    intervals = obstacleMapFuture{iInObatacleMap}(:,1:2);
                    % 要检查的值
                    value = sList(iterInSList);
                    % 计算到每个区间的距离
                    minDistance = CalibrationVars.d_max;
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
                        safeFlag=0;
                        break;
                    else
                        if minDistanceInSList<minDistance
                            minDistanceInSList=minDistance;
                        end
                    end
                    %% 判断末状态与前车的安全距离，是否满足要求
                    if iInObatacleMap==length(obstacleMapFuture)
                        cipvDistance = CalibrationVars.d_max;
                        cipvSpeed=0;
                        for i = 1:size(intervals, 1)
                            if value < intervals(i, 1)
                                frontDistance = intervals(i, 1) - value;
                                frontSpeed=obstacleMapFuture{iInObatacleMap}(i,3);
                            else
                                frontDistance=CalibrationVars.d_max;
                                frontSpeed=0;
                            end
                            % 保存与前车的最小距离
                            if frontDistance < cipvDistance
                                cipvDistance = frontDistance;
                                cipvSpeed=frontSpeed;
                            end
                        end
                        % 判断值是否不满足跟车安全距离
                        if cipvDistance<CalibrationVars.d_max && ...,
                                cipvDistance<CalibrationVars.d_safe+stateNew.speed*CalibrationVars.t_re+max(0,(-stateNew.speed.^2+cipvSpeed.^2)/(2*decel))% 存在前车 && 不满足跟车安全距离
                            safeFlag=0;
                            break;
                        end
                    end
                end
            end
            state.minDistance=minDistanceInSList;
        end
    end
end
%% 生成子节点
if safeFlag==1
    Generation=node(n).Generation+1;%子节点辈分，根节点为0
    actionTillState=node(n).actionTillState;
    actionTillState(Generation,:)=action;
    numOfnodes=numOfnodes+1;
    nodeIndexNew=numOfnodes;%新节点索引
    node(nodeIndexNew).state=stateNew;
    %         node(nodeIndexNew).linkMap=linkMapNew;
    node(nodeIndexNew).parent=n;
    node(nodeIndexNew).actionReward=reward(node(n).state,stateNew,CalibrationVars,v_maxVehicle); % changed
    node(nodeIndexNew).Generation=Generation;
    node(nodeIndexNew).actionTillState=actionTillState;
    %         node=[node,treenode(stateNew,actionTillState,linkMapNew,beBorn,isFullyExpanded,parent,numVisits,totalReward,actionReward,children,Generation)];
    % node(nodeIndexNew)=treenode(stateNew,actionTillState,linkMapNew,beBorn,isFullyExpanded,parent,numVisits,totalReward,children,Generation);%生成子节点
    %更新父节点孩子及标志位
    %         node(n).children=[node(n).children nodeIndexNew];
    node(n).children(nnz(node(n).children)+1)=nodeIndexNew;
    if nnz(node(n).beBorn)==0
        node(n).isFullyExpanded=1;
    end
end
end

% switch action
%     case 1
%         speedOfVehNew=decel*stepLength*p_actionTime+state.speed;
%     case 2
%         speedOfVehNew=stepLength*p_actionTime+state.speed;
%     case 3
%         speedOfVehNew=state.speed;
%
%     case 4
%         speedOfVehNew=stepLength*p_actionTime+state.speed;
%
%     case 5
%         speedOfVehNew=stepLength*p_actionTime+state.speed;
%
% end
% stateNew.speed = middle(speedOfVehNew,v_max,v_min_decel);
