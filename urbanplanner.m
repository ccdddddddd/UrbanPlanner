function traj = urbanplanner(BasicsInfo,ObjectsInfo,ReferenceLineInfo,GlobVars,Parameter,CalibrationVars)
w_veh = Parameter.w_veh; %车宽
l_veh = Parameter.l_veh; %车长
turningRadius = Parameter.turningRadius;
%GlobVars
ischanginglanes = GlobVars.urbanPlanner.ischanginglanes;
isreplanPath = GlobVars.urbanPlanner.isreplanPath;
changLaneStart_s = GlobVars.urbanPlanner.changLaneStart_s;
changLaneEnd_s = GlobVars.urbanPlanner.changLaneEnd_s;
changLanePara = GlobVars.urbanPlanner.changLanePara;
replanStart_s = GlobVars.urbanPlanner.replanStart_s;
replanEnd_s = GlobVars.urbanPlanner.replanEnd_s;
replanLSequcence = GlobVars.urbanPlanner.replanLSequcence;
curTargetLaneIndex = GlobVars.urbanPlanner.curTargetLaneIndex;
laneChangeDirection = GlobVars.urbanPlanner.laneChangeDirection;
%CalibrationVars
pathBuffer = CalibrationVars.urbanPlanner.pathBuffer;
%入参
speed = BasicsInfo.speed; %车速
acce = BasicsInfo.acce; %车速
v_max = BasicsInfo.v_max; %期望车速
veh_s = BasicsInfo.s; %frenet 坐标
veh_l = BasicsInfo.l; %frenet 坐标
veh_psi = BasicsInfo.psi; %frenet 坐标

BasicsInfo.currentLaneIndex = 1; %当前车道序号
targetLaneIndex = BasicsInfo.targetLaneIndex;
currentLaneIndex = BasicsInfo.currentLaneIndex;

lineCurLane_s = ReferenceLineInfo.currentLane.sSequcence;
lineCurLane_k = ReferenceLineInfo.currentLane.kSequcence;
lineCurLane_vlimit = ReferenceLineInfo.currentLane.v_maxSequcence;
% lineCurRightSide_s = ReferenceLineInfo.currentLane.rightSideline_s;
% lineCurRightSide_l = ReferenceLineInfo.currentLane.rightSideline_l;
lineRightLane_s = ReferenceLineInfo.rightLane.s;
lineRightLane_l = ReferenceLineInfo.rightLane.l;
lineRightLane_k = ReferenceLineInfo.rightLane.k;
lineleftLane_s = ReferenceLineInfo.leftLane.s;
lineleftLane_l = ReferenceLineInfo.leftLane.l;
lineleftLane_k = ReferenceLineInfo.leftLane.k;
%规划起点
% dt = 0.1;
% v_0 = speed+acce*dt;
% if v_0 < 0
%     v_0 = 0;
%     t_veh2stop = (0-speed)/acce;
%     advanceDistance = speed*t_veh2stop+0.5*acce*t_veh2stop^2;
% else
%     advanceDistance = speed*dt+0.5*acce*dt^2;
% end
% s_0 = s+advanceDistance*cosd(90-psi);
% l_0 = l+advanceDistance*sind(90-psi);
% a_0 = acce;
% psi_0 = psi;
v_0 = speed;
s_0 = veh_s;
l_0 = veh_l;
a_0 = acce;
psi_0 = veh_psi;
%初始化
s_nearestOnLine = s_0;
% l_nearestOnLine = l_0;
sSequcence = lineCurLane_s;
rSequcence = lineCurLane_k;
v_maxSequcence = lineCurLane_vlimit;
%
if ischanginglanes
    %计算与换道路径偏差
    pathLine = piecewisePolynomial('laneChange',s_0,laneChangeDirection,...
        lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
        changLaneEnd_s,changLaneStart_s,changLanePara,...
        replanStart_s,replanEnd_s,replanLSequcence);
    [s_nearestOnLine,~,dis2line] = calculateProjectionPoint(pathLine,s_0,l_0);
    if dis2line > 0.5%与现有换道路径偏差过大
        if curTargetLaneIndex == currentLaneIndex %位于目标车道
            %重规划2目标车道------------------------------------------------
            if laneChangeDirection == 1
                lineTargLane_s = lineleftLane_s;
                lineTargLane_l = lineleftLane_l;
                lineTargLane_k = lineleftLane_k;
            else
                lineTargLane_s = lineRightLane_s;
                lineTargLane_l = lineRightLane_l;
                lineTargLane_k = lineRightLane_k;
            end
            s_end = min(3*v_0+10,lineTargLane_s(end));
            l_end = interp1(lineTargLane_s, lineTargLane_l, s_end);
            index_targLane = find(lineTargLane_s >= s_end,1);
            slope = (lineTargLane_l(index_targLane) - lineTargLane_l(index_targLane-1))/(lineTargLane_s(index_targLane)-lineTargLane_s(index_targLane-1));
            headingEnd = atand(slope);
            offsetLeft2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
            offsetRight2CurrentLane = offsetLeft2CurrentLane;
            [lSequcence,~] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineTargLane_s,lineTargLane_l,lineTargLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
    CalibrationVars,Parameter);
            replanStart_s = s_0;
            replanEnd_s = s_end;
            replanLSequcence = lSequcence;
            isreplanPath = 2;%2目标车道
            pathLinePart1_s = linspace(replanStart_s,replanEnd_s,length(replanLSequcence));
            pathLinePart1_l = replanLSequcence;
            pathLine = spline([pathLinePart1_s,replanEnd_s+50], [pathLinePart1_l,0]);
            %速度规划-------------------------------------------------------
            offset = 0.5*w_veh+pathBuffer;
            pathStart_s = s_0;
            obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%             sSequcence = [0,1];
%             rSequcence = [0,0];
%             v_maxSequcence = [v_max,v_max];
            [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
            [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
            sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
            sMaxSequenceMCTS=sMinSequenceMCTS+999;
            v_end = speedList(end);
            s_end = sMCTS(end);
            [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
            %---------------------------------------------------------------
            if exitflag %速度ok  
                %生成轨迹
                trajectoryType = 1;
            else
                %生成紧急制动轨迹
                trajectoryType = 2;
            end
        else
            %重换道+速度规划------------------------------------------------
%             laneChangeDirection = -sign(targetLaneIndex-currentLaneIndex);
%             curTargetLaneIndex = currentLaneIndex+sign(targetLaneIndex-currentLaneIndex);
            if laneChangeDirection == 1
                lineTargLane_s = lineleftLane_s;
                lineTargLane_l = lineleftLane_l;
            else
                lineTargLane_s = lineRightLane_s;
                lineTargLane_l = lineRightLane_l;
            end
            offsetTarget2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
            slopes = zeros(1, length(lineTargLane_s));
            for i = 1:length(x)-1
                slopes(i) = (lineTargLane_l(i+1) - lineTargLane_l(i)) / (lineTargLane_s(i+1) - lineTargLane_s(i));
            end
            slopes(end) = slopes(end-1);
            headingTargetLaneSequcence = atand(slopes);
            curPathLine = piecewisePolynomial('currentLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            tarPathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            offset = 0.5*w_veh+pathBuffer;
            pathStart_s = s_0;
            obstacleMap=obstaclegraph(ObjectsInfo,curPathLine,offset,pathStart_s,pathStart_s+150);
            obstacleMapTargetLane=obstaclegraph(ObjectsInfo,tarPathLine,offset,pathStart_s,pathStart_s+150);
            [laneChange_s_end,para,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
                ,headingTargetLaneSequcence,90-psi_0,obstacleMap,obstacleMapTargetLane,CalibrationVars,Parameter,plotFlag,rectangles,rectanglesTargetLane);
            if laneChangeDec%换道ok
                ischanginglanes = 1;
                changLanePara = para;
                changLaneStart_s = s_0;
                changLaneEnd_s = laneChange_s_end;
                pathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
                %速度规划---------------------------------------------------
                offset = 0.5*w_veh+pathBuffer;
                pathStart_s = s_0;
                obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%                 sSequcence =
%                 rSequcence =
%                 v_maxSequcence =
                [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
                [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
                sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
                sMaxSequenceMCTS=sMinSequenceMCTS+999;
                v_end = speedList(end);
                s_end = sMCTS(end);
                [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
                if exitflag == 1%速度规划ok
                    %生成轨迹
                    trajectoryType = 1;
                end
                if laneChangeDec == 0 || exitflag == 0%换道不ok || %速度规划不ok--------------------------------
                    ischanginglanes = 0;
                    changLanePara = [];
                    %重规划原车道+ 速度规划---------------------------------
                    s_end = min(3*v_0+10,lineCurLane_s(end));
                    l_end = 0;
                    headingEnd = 0;
                    offsetLeft2CurrentLane = interp1(lineleftLane_s, lineleftLane_l, s_0);
                    offsetRight2CurrentLane = interp1(lineRightLane_s, lineRightLane_l, s_0);
                    [lSequcence,~] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineCurLane_s,lineCurLane_l,lineCurLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
                        CalibrationVars,Parameter);
                    replanStart_s = s_0;
                    replanEnd_s = s_end;
                    replanLSequcence = lSequcence;
                    isreplanPath = 1;%1原车道
                    pathLine = piecewisePolynomial('replanPath',s_0,laneChangeDirection,...
                        lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                        changLaneEnd_s,changLaneStart_s,changLanePara,...
                        replanStart_s,replanEnd_s,replanLSequcence);
                    %速度规划
                    offset = 0.5*w_veh+pathBuffer;
                    pathStart_s = s_0;
                    obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%                     sSequcence =
%                     rSequcence =
%                     v_maxSequcence =
                    [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
                    [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
                    sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
                    sMaxSequenceMCTS=sMinSequenceMCTS+999;
                    v_end = speedList(end);
                    s_end = sMCTS(end);
                    [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
                    if exitflag ==1 %速度ok
                        % 生成轨迹
                        trajectoryType = 1;
                    else
                        %紧急制动轨迹
                        trajectoryType = 2;
                    end
                end
            end
        end
    else
        %旧换道路径速度规划--------------------------------------------------
        offset = 0.5*w_veh+pathBuffer;
        fprime = fnder(pathLine,1);%一阶导
        pathStart_s = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), s_nearestOnLine)+pathLine.breaks(1);%s_0的投影点在线上的s坐标
        obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%         sSequcence =
%         rSequcence =
%         v_maxSequcence =
        [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
        [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
        sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_end = speedList(end);
        s_end = sMCTS(end);
        [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
        if exitflag == 1%速度规划成功
            %生成轨迹
            trajectoryType = 1;
        elseif curTargetLaneIndex == currentLaneIndex  %位于目标车道
            %生成紧急制动轨迹
            trajectoryType = 2;
        else
            %重换道+速度规划------------------------------------------------
            %             laneChangeDirection = -sign(targetLaneIndex-currentLaneIndex);
            %             curTargetLaneIndex = currentLaneIndex+sign(targetLaneIndex-currentLaneIndex);
            if laneChangeDirection == 1
                lineTargLane_s = lineleftLane_s;
                lineTargLane_l = lineleftLane_l;
            else
                lineTargLane_s = lineRightLane_s;
                lineTargLane_l = lineRightLane_l;
            end
            offsetTarget2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
            slopes = zeros(1, length(lineTargLane_s));
            for i = 1:length(x)-1
                slopes(i) = (lineTargLane_l(i+1) - lineTargLane_l(i)) / (lineTargLane_s(i+1) - lineTargLane_s(i));
            end
            slopes(end) = slopes(end-1);
            headingTargetLaneSequcence = atand(slopes);
            curPathLine = piecewisePolynomial('currentLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            tarPathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            offset = 0.5*w_veh+pathBuffer;
            pathStart_s = s_0;
            obstacleMap=obstaclegraph(ObjectsInfo,curPathLine,offset,pathStart_s,pathStart_s+150);
            obstacleMapTargetLane=obstaclegraph(ObjectsInfo,tarPathLine,offset,pathStart_s,pathStart_s+150);
            [laneChange_s_end,para,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
                ,headingTargetLaneSequcence,90-psi_0,obstacleMap,obstacleMapTargetLane,CalibrationVars,Parameter,plotFlag,rectangles,rectanglesTargetLane);
            if laneChangeDec%换道ok
                ischanginglanes = 1;
                changLanePara = para;
                changLaneStart_s = s_0;
                changLaneEnd_s = laneChange_s_end;
                pathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    replanStart_s,replanEnd_s,replanLSequcence);
                %速度规划---------------------------------------------------
                offset = 0.5*w_veh+pathBuffer;
                pathStart_s = s_0;
                obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%                 sSequcence =
%                 rSequcence =
%                 v_maxSequcence =
                [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
                [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
                sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
                sMaxSequenceMCTS=sMinSequenceMCTS+999;
                v_end = speedList(end);
                s_end = sMCTS(end);
                [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
                if exitflag == 1%速度规划ok
                    %生成轨迹
                end
                if laneChangeDec == 0 || exitflag == 0%换道不ok || %速度规划不ok--------------------------------
                    ischanginglanes = 0;
                    changLanePara = [];
                    %重规划原车道+ 速度规划---------------------------------
                    s_end = min(3*v_0+10,lineCurLane_s(end));
                    l_end = 0;
                    headingEnd = 0;
                    offsetLeft2CurrentLane = interp1(lineleftLane_s, lineleftLane_l, s_0);
                    offsetRight2CurrentLane = interp1(lineRightLane_s, lineRightLane_l, s_0);
                    [lSequcence,~] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineCurLane_s,lineCurLane_l,lineCurLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
                        CalibrationVars,Parameter);
                    replanStart_s = s_0;
                    replanEnd_s = s_end;
                    replanLSequcence = lSequcence;
                    isreplanPath = 1;%1原车道
                    pathLine = piecewisePolynomial('replanPath',s_0,laneChangeDirection,...
                        lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                        changLaneEnd_s,changLaneStart_s,changLanePara,...
                        replanStart_s,replanEnd_s,replanLSequcence);
                    %速度规划
                    offset = 0.5*w_veh+pathBuffer;
                    pathStart_s = s_0;
                    obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%                     sSequcence =
%                     rSequcence =
%                     v_maxSequcence =
                    [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
                    [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
                    sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
                    sMaxSequenceMCTS=sMinSequenceMCTS+999;
                    v_end = speedList(end);
                    s_end = sMCTS(end);
                    [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
                    if exitflag ==1 %速度ok
                        % 生成轨迹
                        trajectoryType = 1;
                    else
                        %紧急制动轨迹
                        trajectoryType = 2;
                    end
                end
            end
        end
    end
elseif isreplanPath == 2 %重规划目标车道中
    pathLine = piecewisePolynomial('replanPath',s_0,laneChangeDirection,...
        lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
        changLaneEnd_s,changLaneStart_s,changLanePara,...
        replanStart_s,replanEnd_s,replanLSequcence);
    [s_nearestOnLine,~,~] = calculateProjectionPoint(pathLine,s_0,l_0);
    %速度规划
    offset = 0.5*w_veh+pathBuffer;
    fprime = fnder(pathLine,1);%一阶导
    pathStart_s = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), s_nearestOnLine)+pathLine.breaks(1);%s_0的投影点在线上的s坐标
    obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%     sSequcence =
%     rSequcence =
%     v_maxSequcence =
    [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
    [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
    sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
    sMaxSequenceMCTS=sMinSequenceMCTS+999;
    v_end = speedList(end);
    s_end = sMCTS(end);
    [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
    if exitflag == 1%速度规划ok
        %生成轨迹
        trajectoryType = 1;
    else
        %紧急制动轨迹
        trajectoryType = 2;
    end 
else
    if currentLaneIndex~=targetLaneIndex %需换道
        %换道路径-----------------------------------------------------------
        laneChangeDirection = -sign(targetLaneIndex-currentLaneIndex);
        curTargetLaneIndex = currentLaneIndex+sign(targetLaneIndex-currentLaneIndex);
        if laneChangeDirection == 1
            lineTargLane_s = lineleftLane_s;
            lineTargLane_l = lineleftLane_l;
        else
            lineTargLane_s = lineRightLane_s;
            lineTargLane_l = lineRightLane_l;
        end
        offsetTarget2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
        slopes = zeros(1, length(lineTargLane_s));
        for i = 1:length(x)-1
            slopes(i) = (lineTargLane_l(i+1) - lineTargLane_l(i)) / (lineTargLane_s(i+1) - lineTargLane_s(i));
        end
        slopes(end) = slopes(end-1);
        headingTargetLaneSequcence = atand(slopes);
        curPathLine = piecewisePolynomial('currentLane',s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            replanStart_s,replanEnd_s,replanLSequcence);
        tarPathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            replanStart_s,replanEnd_s,replanLSequcence);
        offset = 0.5*w_veh+pathBuffer;
        pathStart_s = s_0;
        obstacleMap=obstaclegraph(ObjectsInfo,curPathLine,offset,pathStart_s,pathStart_s+150);
        obstacleMapTargetLane=obstaclegraph(ObjectsInfo,tarPathLine,offset,pathStart_s,pathStart_s+150);
        [laneChange_s_end,para,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
            ,headingTargetLaneSequcence,90-psi_0,obstacleMap,obstacleMapTargetLane,CalibrationVars,Parameter,plotFlag,rectangles,rectanglesTargetLane);
        if laneChangeDec == 1%换道路径ok
            ischanginglanes = 1;
            changLanePara = para;
            changLaneStart_s = s_0;
            changLaneEnd_s = laneChange_s_end;
            pathLine = piecewisePolynomial('targetLane',s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            %速度规划
            offset = 0.5*w_veh+pathBuffer;
            pathStart_s = s_0;
            obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%             sSequcence =
%             rSequcence =
%             v_maxSequcence =
            [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
            [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
            sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
            sMaxSequenceMCTS=sMinSequenceMCTS+999;
            v_end = speedList(end);
            s_end = sMCTS(end);
            [jOptSequence,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
            if exitflag ==1 %速度规划ok
                %生成轨迹
                trajectoryType = 1;
            else
                %紧急制动轨迹
                trajectoryType = 2;
            end
        else
            ischanginglanes = 0;
            changLanePara = [];
            changLaneStart_s = [];
            changLaneEnd_s = [];
        end
    end
    if ischanginglanes~=1 && isreplanPath == 1 %无需换道或换道不ok且重规划中
        pathLine = piecewisePolynomial('replanPath',s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            replanStart_s,replanEnd_s,replanLSequcence);
        [s_nearestOnLine,~,~] = calculateProjectionPoint(pathLine,s_0,l_0);
        %速度规划
        offset = 0.5*w_veh+pathBuffer;
        fprime = fnder(pathLine,1);%一阶导
        pathStart_s = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), s_nearestOnLine)+pathLine.breaks(1);%s_0的投影点在线上的s坐标
        obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%         sSequcence =
%         rSequcence =
%         v_maxSequcence =
        [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
        [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
        sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_end = speedList(end);
        s_end = sMCTS(end);
        [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
        if exitflag == 1%速度规划ok
            %生成轨迹
            trajectoryType = 1;
        else
            %紧急制动轨迹
            trajectoryType = 2;
        end

    elseif ischanginglanes~=1 &&  abs(l_0)>0.3 || abs(90-psi_0)>10 %无需换道或换道不ok且需重规划
        %重规划原车道+ 速度规划---------------------------------
        s_end = min(3*v_0+10,lineCurLane_s(end));
        l_end = 0;
        headingEnd = 0;
        offsetLeft2CurrentLane = interp1(lineleftLane_s, lineleftLane_l, s_0);
        offsetRight2CurrentLane = interp1(lineRightLane_s, lineRightLane_l, s_0);
        [lSequcence,~] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineCurLane_s,lineCurLane_l,lineCurLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
            CalibrationVars,Parameter);
        replanStart_s = s_0;
        replanEnd_s = s_end;
        replanLSequcence = lSequcence;
        isreplanPath = 1;%1原车道
        pathLine = piecewisePolynomial('replanPath',s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            replanStart_s,replanEnd_s,replanLSequcence);
        %速度规划
        offset = 0.5*w_veh+pathBuffer;
        pathStart_s = s_0;
        obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%         sSequcence =
%         rSequcence =
%         v_maxSequcence =
        [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
        [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
        sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_end = speedList(end);
        s_end = sMCTS(end);
        [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
        if exitflag%速度规划ok
            %生成轨迹
            trajectoryType = 1;
        else
            %紧急制动轨迹
            trajectoryType = 2;
        end
    else 
        pathLine = piecewisePolynomial('currentLane',s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            replanStart_s,replanEnd_s,replanLSequcence);
        %速度规划
        offset = 0.5*w_veh+pathBuffer;
        pathStart_s = s_0;
        obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathStart_s+150);
%         sSequcence =
%         rSequcence =
%         v_maxSequcence =
        [~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,pathStart_s,sSequcence,rSequcence,v_max,v_maxSequcence,CalibrationVars);
        [sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max);
        sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_end = speedList(end);
        s_end = sMCTS(end);
        [~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibrationVars);
        if exitflag%速度规划ok
            %生成轨迹
            trajectoryType = 1;
        else
            %紧急制动轨迹
            trajectoryType = 2;
        end
        
    end
end
%轨迹生成
if trajectoryType == 2
    t_list = 0:0.1:5;
    aOptSequence = zeros(1,length(t_list))-4;
    vOptSequence = v_0+aOptSequence.*t_list;
    sOptSequence = pathStart_s +v_0.*t_list+0.5*aOptSequence.*t_list.^2;
end
numOfPoint = length(sOptSequence);
s = zeros(1,numOfPoint);
s_dot = zeros(1,numOfPoint);
s_ddot = zeros(1,numOfPoint);
l = zeros(1,numOfPoint);
l_dot = zeros(1,numOfPoint);
l_ddot = zeros(1,numOfPoint);
dl = zeros(1,numOfPoint);
ddl = zeros(1,numOfPoint);
for i=1:length(sOptSequence)
    if i==1
        arclength = 0;
        pre_s = s_nearestOnLine;
    else
        arclength = sOptSequence(i)-sOptSequence(i-1);
        pre_s = s(i-1);
    end
    fprime = fnder(pathLine,1);%一阶导
    fprime2 = fnder(pathLine,2);%二阶导
    fun_a = @(x)sqrt(1+(ppval(fprime,x)).^2);
    fun_x=@(x)trapz(linspace(pre_s,x,50),fun_a(linspace(pre_s,x,50)));
    [cur_s,~,~] = fzero(@(x)fun_x(x)-arclength,[pre_s pathLine.breaks(end)]);
    s(i) = cur_s;
    l(i) = ppval(pathLine,cur_s);
    dl(i) = ppval(fprime,cur_s);
    ddl(i) = ppval(fprime2,cur_s);
    s_dot(i) = vOptSequence(i)*cosd(atand(dl(i)));
    s_ddot(i) = aOptSequence(i)*cosd(atand(dl(i)));
    l_dot(i) = vOptSequence(i)*sind(atand(dl(i)));
    l_ddot(i) = aOptSequence(i)*sind(atand(dl(i)));
end
traj.s = s;
traj.s_dot = s_dot;
traj.s_ddot = s_ddot;
traj.l = l;
traj.l_dot = l_dot;
traj.l_ddot = l_ddot;
traj.dl = dl;
traj.ddl = ddl;
%GlobVars
GlobVars.urbanPlanner.ischanginglanes = ischanginglanes;
GlobVars.urbanPlanner.isreplanPath = isreplanPath;
GlobVars.urbanPlanner.changLaneStart_s = changLaneStart_s;
GlobVars.urbanPlanner.changLaneEnd_s = changLaneEnd_s;
GlobVars.urbanPlanner.changLanePara = changLanePara;
GlobVars.urbanPlanner.replanStart_s = replanStart_s;
GlobVars.urbanPlanner.replanEnd_s = replanEnd_s;
GlobVars.urbanPlanner.replanLSequcence = replanLSequcence;
GlobVars.urbanPlanner.curTargetLaneIndex = curTargetLaneIndex;
GlobVars.urbanPlanner.laneChangeDirection = laneChangeDirection;
end




