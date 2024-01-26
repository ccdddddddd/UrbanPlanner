function [traj,trajectoryType,GlobVars] = urbanplanner(BasicsInfo,ObstacleInfo,LaneLineInfo,GlobVars,Parameter,CalibrationVars)
w_veh = Parameter.w_veh; %车宽
l_veh = Parameter.l_veh; %车长
turningRadius = Parameter.turningRadius;
steerRatio = Parameter.steerRatio;
wheelBase = Parameter.wheelBase;
%GlobVars
ischanginglanes = GlobVars.urbanPlanner.ischanginglanes;
isreplanPath = GlobVars.urbanPlanner.isreplanPath;
changLaneStart_s = GlobVars.urbanPlanner.changLaneStart_s;
changLaneEnd_s = GlobVars.urbanPlanner.changLaneEnd_s;
changLanePara = GlobVars.urbanPlanner.changLanePara;
% replanStart_s = GlobVars.urbanPlanner.replanStart_s;
% replanEnd_s = GlobVars.urbanPlanner.replanEnd_s;
% replanLSequcence = GlobVars.urbanPlanner.replanLSequcence;
replanePath_x = GlobVars.urbanPlanner.replanePath_x;
replanePath_y = GlobVars.urbanPlanner.replanePath_y;
curTargetLaneIndex = GlobVars.urbanPlanner.curTargetLaneIndex;
laneChangeDirection = GlobVars.urbanPlanner.laneChangeDirection;
%CalibrationVars
pathBuffer = CalibrationVars.urbanPlanner.pathBuffer;
CalibLCPath = CalibrationVars.CalibLCPath;
CalibReplanPath = CalibrationVars.CalibReplanPath;
%入参
v_max = BasicsInfo.v_max; %期望车速
vehicleState.speed = BasicsInfo.speed; 
vehicleState.acce = BasicsInfo.acce; 
vehicleState.x = BasicsInfo.x; 
vehicleState.y = BasicsInfo.y; 
vehicleState.theta = BasicsInfo.theta; 
vehicleState.k = tand(BasicsInfo.SteerAngle/steerRatio)/wheelBase;
targetLaneIndex = BasicsInfo.targetLaneIndex;
currentLaneIndex = BasicsInfo.currentLaneIndex;
%参考线生成
[ReferenceLineInfo,refLine] = referencelinegeneration(LaneLineInfo,v_max,CalibrationVars);
lineCurLane_s = ReferenceLineInfo.currentLane.sSequcence;
lineCurLane_k = ReferenceLineInfo.currentLane.kSequcence;
lineCurLane_vlimit = ReferenceLineInfo.currentLane.v_maxSequcence;
lineRightLane_s = ReferenceLineInfo.rightLane.s;
lineRightLane_l = ReferenceLineInfo.rightLane.l;
lineRightLane_dl = ReferenceLineInfo.rightLane.dl;
lineRightLane_k = ReferenceLineInfo.rightLane.k;
lineleftLane_s = ReferenceLineInfo.leftLane.s;
lineleftLane_l = ReferenceLineInfo.leftLane.l;
lineleftLane_dl = ReferenceLineInfo.leftLane.dl;
lineleftLane_k = ReferenceLineInfo.leftLane.k;
%规划起点
vehFrenetState = global2frenet(refLine,[vehicleState.x,vehicleState.y,vehicleState.theta,vehicleState.k,vehicleState.speed,vehicleState.acce]);
v_0 = vehicleState.speed;
s_0 = vehFrenetState(1);
l_0 = vehFrenetState(4);
a_0 = vehicleState.acce;
theta_r = tangentAngle(refLine,s_0);
psi_0 = rad2deg(vehicleState.theta - theta_r);
%obstacle2Frenet
ObstaclesFrenetState = obstacle2frenetstate(refLine,ObstacleInfo);
if ischanginglanes
    %计算与换道路径偏差
    pathLine = piecewisePolynomial('laneChange',refLine,s_0,laneChangeDirection,...
        lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
        changLaneEnd_s,changLaneStart_s,changLanePara,...
        0,0,0);
    [~,~,theta_line,dis2line] = calculateProjectionPoint(pathLine,s_0,l_0);
    if dis2line > 0.3 || abs(psi_0-theta_line) > 10 %与现有换道路径偏差过大
        if curTargetLaneIndex == currentLaneIndex %位于目标车道
            %重规划2目标车道------------------------------------------------
            ischanginglanes = 0;
            changLanePara = [];
            changLaneStart_s = 0;
            changLaneEnd_s = 0;
            if laneChangeDirection == 1
                lineTargLane_s = lineleftLane_s;
                lineTargLane_l = lineleftLane_l;
                lineTargLane_dl = lineleftLane_dl;
                lineTargLane_k = lineleftLane_k;
            else
                lineTargLane_s = lineRightLane_s;
                lineTargLane_l = lineRightLane_l;
                lineTargLane_dl = lineRightLane_dl;
                lineTargLane_k = lineRightLane_k;
            end
            s_end = min(s_0+3*v_0+10,lineTargLane_s(end));
            l_end = interp1(lineTargLane_s, lineTargLane_l, s_end);
%             index_targLane = find(lineTargLane_s >= s_end,1);
%             slope = (lineTargLane_l(index_targLane) - lineTargLane_l(index_targLane-1))/(lineTargLane_s(index_targLane)-lineTargLane_s(index_targLane-1));
%             headingEnd = atand(slope);
            headingEnd = atand(lineTargLane_dl);
%             offsetLeft2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
%             offsetRight2CurrentLane = offsetLeft2CurrentLane; % To be changed
            if ~isempty(lineleftLane_s)
                offsetLeft2CurrentLane = abs(interp1(lineleftLane_s, lineleftLane_l, s_0+0.1));
            else
                offsetLeft2CurrentLane = 3.2;
            end
            if ~isempty(lineRightLane_s)
                offsetRight2CurrentLane = abs(interp1(lineRightLane_s, lineRightLane_l, s_0+0.1));
            else
                offsetRight2CurrentLane = 3.2;
            end
            [lSequcence,replan_exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineTargLane_s,lineTargLane_l,lineTargLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
    CalibReplanPath,Parameter);
            if replan_exitflag == 1 %重规划路径ok
                isreplanPath = 2;%2目标车道
            elseif replan_exitflag == -1
                isreplanPath = 3; %重规划路径超边界
            end
            s = linspace(s_0,s_end,length(lSequcence))';
            l = lSequcence;
            globalStates = frenet2global(refLine,[s,zeros(length(s),1),zeros(length(s),1),l,zeros(length(s),1),zeros(length(s),1)]);
            replanePath_x = globalStates(:,1);
            replanePath_y = globalStates(:,2);

            replanStart_s = s_0;
            replanEnd_s = s_end;
            replanLSequcence = lSequcence;
            pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
%             pathLinePart1_s = linspace(replanStart_s,replanEnd_s,length(replanLSequcence));
%             pathLinePart1_l = replanLSequcence;
%             pathLine = spline([pathLinePart1_s,replanEnd_s+50], [pathLinePart1_l,0]);
            %速度规划-------------------------------------------------------
            [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
            %---------------------------------------------------------------
            if exitflag==1 %速度ok
                trajectoryType = 1;%生成轨迹
            else
                trajectoryType = -1;%生成紧急制动轨迹
            end
        else
            %重换道+速度规划------------------------------------------------
            if laneChangeDirection == 1
                lineTargLane_s = lineleftLane_s;
                lineTargLane_l = lineleftLane_l;
            else
                lineTargLane_s = lineRightLane_s;
                lineTargLane_l = lineRightLane_l;
            end
            offsetTarget2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
            slopes = zeros(1, length(lineTargLane_s));
            for i = 1:length(lineTargLane_s)-1
                slopes(i) = (lineTargLane_l(i+1) - lineTargLane_l(i)) / (lineTargLane_s(i+1) - lineTargLane_s(i));
            end
            slopes(end) = slopes(end-1);
            headingTargetLaneSequcence = atand(slopes);
            tarPathLine = piecewisePolynomial('targetLane',refLine,s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                0,0,0);
            offset = 0.5*w_veh+pathBuffer;
            obstacleMap=obstacleSTgraph(ObstacleInfo,refLine,offset);
            obstacleMapTargetLane=obstaclegraph(ObstaclesFrenetState,tarPathLine,offset);
            [para,laneChange_s_end,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
                ,headingTargetLaneSequcence,psi_0,obstacleMap,obstacleMapTargetLane,CalibLCPath,Parameter,plotFlag,rectangles,rectanglesTargetLane);
            if laneChangeDec%换道ok
                ischanginglanes = 1;
                changLanePara = para;
                changLaneStart_s = s_0;
                changLaneEnd_s = laneChange_s_end;
                pathLine = piecewisePolynomial('laneChange',refLine,s_0,laneChangeDirection,... % is changed
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                0,0,0);
                %速度规划---------------------------------------------------
                [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
                if exitflag == 1%速度规划ok
                    %生成轨迹
                    trajectoryType = 2;
                end
            end
            if laneChangeDec == 0 || exitflag == 0%换道不ok || %速度规划不ok--------------------------------
                ischanginglanes = 0;
                changLanePara = [];
                changLaneStart_s = 0;
                changLaneEnd_s = 0;
                %重规划原车道+ 速度规划---------------------------------
                s_end = min(s_0+3*v_0+10,lineCurLane_s(end)); % to be changed or calibrated
                l_end = 0;
                headingEnd = 0;
                if ~isempty(lineleftLane_s)
                    offsetLeft2CurrentLane = abs(interp1(lineleftLane_s, lineleftLane_l, s_0+0.1));
                else
                    offsetLeft2CurrentLane = 3.2;
                end
                if ~isempty(lineRightLane_s)
                    offsetRight2CurrentLane = abs(interp1(lineRightLane_s, lineRightLane_l, s_0+0.1));
                else
                    offsetRight2CurrentLane = 3.2;
                end
                lineCurLane_l=zeors(1,length(lineCurLane_s));
                [lSequcence,replan_exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineCurLane_s,lineCurLane_l,lineCurLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
                    CalibReplanPath,Parameter); % to be changed: lineCurLane_l=zeors(1,length(lineCurLane_s));
                if replan_exitflag == 1 %重规划路径ok
                    isreplanPath = 1;%1原车道
                elseif replan_exitflag == -1
                    isreplanPath = 3; %重规划路径超边界
                end
                s = linspace(s_0,s_end,length(lSequcence))';
                l = lSequcence;
                globalStates = frenet2global(refLine,[s,zeros(length(s),1),zeros(length(s),1),l,zeros(length(s),1),zeros(length(s),1)]);
                replanePath_x = globalStates(:,1);
                replanePath_y = globalStates(:,2);
                replanStart_s = s_0;
                replanEnd_s = s_end;
                replanLSequcence = lSequcence;
                pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    replanStart_s,replanEnd_s,replanLSequcence);
                %速度规划
                [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
                if exitflag ==1 %速度ok
                    % 生成轨迹
                    trajectoryType = 3;
                else
                    %紧急制动轨迹
                    trajectoryType = -3;
                end
            end 
        end
    else
        %旧换道路径速度规划--------------------------------------------------
        [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
        if exitflag == 1%速度规划成功
            %生成轨迹
            trajectoryType = 4;
            %换道结束判断
            if s_0 >= changLaneEnd_s
                ischanginglanes = 0;
                changLanePara = [];
                changLaneStart_s = 0;
                changLaneEnd_s = 0;
            end
        elseif curTargetLaneIndex == currentLaneIndex  %位于目标车道
            %生成紧急制动轨迹
            trajectoryType = -4;
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
            for i = 1:length(lineTargLane_s)-1
                slopes(i) = (lineTargLane_l(i+1) - lineTargLane_l(i)) / (lineTargLane_s(i+1) - lineTargLane_s(i));
            end
            slopes(end) = slopes(end-1);
            headingTargetLaneSequcence = atand(slopes);
            tarPathLine = piecewisePolynomial('targetLane',refLine,s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                0,0,0);
            offset = 0.5*w_veh+pathBuffer;
            obstacleMap=obstacleSTgraph(ObstacleInfo,refLine,offset);
            obstacleMapTargetLane=obstaclegraph(ObstaclesFrenetState,tarPathLine,offset);
            [para,laneChange_s_end,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
                ,headingTargetLaneSequcence,psi_0,obstacleMap,obstacleMapTargetLane,CalibLCPath,Parameter,plotFlag,rectangles,rectanglesTargetLane); 
            %plotFlag,rectangles,rectanglesTargetLane需要赋值
            if laneChangeDec%换道ok
                ischanginglanes = 1;
                changLanePara = para;
                changLaneStart_s = s_0;
                changLaneEnd_s = laneChange_s_end;
                pathLine = piecewisePolynomial('targetLane',refLine,s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    0,0,0);
                %速度规划---------------------------------------------------
                [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
                if exitflag == 1%速度规划ok
                    %生成轨迹
                    trajectoryType = 5;
                end
            end
            if laneChangeDec == 0 || exitflag == 0%换道不ok || %速度规划不ok--------------------------------
                ischanginglanes = 0;
                changLanePara = [];
                changLaneStart_s = 0;
                changLaneEnd_s = 0;
                %重规划原车道+ 速度规划---------------------------------
                s_end = min(s_0+3*v_0+10,lineCurLane_s(end));
                l_end = 0;
                headingEnd = 0;
                if ~isempty(lineleftLane_s)
                    offsetLeft2CurrentLane = abs(interp1(lineleftLane_s, lineleftLane_l, s_0+0.1));
                else
                    offsetLeft2CurrentLane = 3.2;
                end
                if ~isempty(lineRightLane_s)
                    offsetRight2CurrentLane = abs(interp1(lineRightLane_s, lineRightLane_l, s_0+0.1));
                else
                    offsetRight2CurrentLane = 3.2;
                end
                lineCurLane_l=zeors(1,length(lineCurLane_s));
                [lSequcence,replan_exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,lineCurLane_s,lineCurLane_l,lineCurLane_k,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
                    CalibReplanPath,Parameter);
                if replan_exitflag == 1 %重规划路径ok
                    isreplanPath = 1;%1原车道
                elseif replan_exitflag == -1
                    isreplanPath = 3; %重规划路径超边界
                end
                s = linspace(s_0,s_end,length(lSequcence))';
                l = lSequcence;
                globalStates = frenet2global(refLine,[s,zeros(length(s),1),zeros(length(s),1),l,zeros(length(s),1),zeros(length(s),1)]);
                replanePath_x = globalStates(:,1);
                replanePath_y = globalStates(:,2);
                replanStart_s = s_0;
                replanEnd_s = s_end;
                replanLSequcence = lSequcence;
                pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    replanStart_s,replanEnd_s,replanLSequcence);
                %速度规划
                [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
                if exitflag ==1 %速度ok
                    % 生成轨迹
                    trajectoryType = 6;
                else
                    %紧急制动轨迹
                    trajectoryType = -6;
                end
            end
        end
    end
% elseif isreplanPath == 2 %重规划目标车道中
%     pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
%         lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
%         changLaneEnd_s,changLaneStart_s,changLanePara,...
%         replanStart_s,replanEnd_s,replanLSequcence);
%     %速度规划
%     [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
%     if exitflag == 1%速度规划ok
%         %生成轨迹
%         trajectoryType = 7;
%     else
%         %紧急制动轨迹
%         trajectoryType = -7;
%     end 
else
    if currentLaneIndex~=targetLaneIndex %需换道
        %换道路径-----------------------------------------------------------
        laneChangeDirection = -sign(targetLaneIndex-currentLaneIndex);
        curTargetLaneIndex = currentLaneIndex+sign(targetLaneIndex-currentLaneIndex);
        if laneChangeDirection == 1
            lineTargLane_s = lineleftLane_s;
            lineTargLane_l = lineleftLane_l;
            lineTargLane_dl = lineleftLane_dl;
        else
            lineTargLane_s = lineRightLane_s;
            lineTargLane_l = lineRightLane_l;
            lineTargLane_dl = lineRightLane_dl;
        end
        offsetTarget2CurrentLane = interp1(lineTargLane_s, lineTargLane_l, s_0);
        headingTargetLaneSequcence = atand(lineTargLane_dl);
        tarPathLine = piecewisePolynomial('targetLane',refLine,s_0,laneChangeDirection,...
            lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
            changLaneEnd_s,changLaneStart_s,changLanePara,...
            0,0,0);
        offset = 0.5*w_veh+pathBuffer;
        obstacleMap=obstacleSTgraph(ObstacleInfo,refLine,offset);
        obstacleMapTargetLane=obstaclegraph(ObstaclesFrenetState,tarPathLine,offset);
        [para,laneChange_s_end,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,lineTargLane_s,lineTargLane_l...,
            ,headingTargetLaneSequcence,psi_0,obstacleMap,obstacleMapTargetLane,CalibLCPath,Parameter,0,[],[]);
        if laneChangeDec == 1%换道路径ok
            ischanginglanes = 1;
            changLanePara = para;
            changLaneStart_s = s_0;
            changLaneEnd_s = laneChange_s_end;
            pathLine = piecewisePolynomial('laneChange',refLine,s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                0,0,0);
            %速度规划
            [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
            if exitflag ==1 %速度规划ok
                %生成轨迹
                trajectoryType = 8;
            else
                ischanginglanes = 0;
                changLanePara = [];
                changLaneStart_s = 0;
                changLaneEnd_s = 0;
            end
        else
            ischanginglanes = 0;
            changLanePara = [];
            changLaneStart_s = 0;
            changLaneEnd_s = 0;
        end
    end
    if ischanginglanes~=1 %无需换道或换道不ok或换道速度规划不ok
        %与当前参考线偏差
        isOverDeviation = abs(l_0)>0.3 || abs(psi_0)>10;
        %与重规划路径偏差
        if isreplanPath == 1 || isreplanPath == 2 %重规划中
            if isOverDeviation == 0 %重规划结束判断
                isreplanPath = 0;
                replanStart_s = 0;
                replanEnd_s = 0;
                replanLSequcence = [];
                replanePath_x = [];
                replanePath_y = [];
            else
                replanLineXY = spline(replanePath_x, replanePath_y);
                [~,~,theta_line,dis2line] = calculateProjectionPoint(replanLineXY,vehicleState.x,vehicleState.y);
                isOverDeviation = dis2line>0.3 || abs(rad2deg(vehicleState.theta)-theta_line)>10;
            end
        end
        if isOverDeviation == 1 %偏差过大
            %重规划回到当前车道大的路径
            s_end = min(s_0+3*v_0+10,lineCurLane_s(end));
            l_end = 0;
            headingEnd = 0;
            if ~isempty(lineleftLane_s)
                offsetLeft2CurrentLane = abs(interp1(lineleftLane_s, lineleftLane_l, s_0+0.1));
            else
                offsetLeft2CurrentLane = 3.2;
            end
            if ~isempty(lineRightLane_s)
                offsetRight2CurrentLane = abs(interp1(lineRightLane_s, lineRightLane_l, s_0+0.1));
            else
                offsetRight2CurrentLane = 3.2;
            end
            replan_sSequcence = lineCurLane_s;
            replan_lRefSequcence = zeros(1,length(lineCurLane_s));
            replan_rSequcence = abs(1./(lineCurLane_k+eps));
            [lSequcence,replan_exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,replan_sSequcence,replan_lRefSequcence,replan_rSequcence,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
                CalibReplanPath,Parameter);
            if replan_exitflag == 1 %重规划路径ok
                isreplanPath = 1;%1原车道
            elseif replan_exitflag == -1
                isreplanPath = 3; %重规划路径超边界
            end
            s = linspace(s_0,s_end,length(lSequcence))';
            l = lSequcence;
            globalStates = frenet2global(refLine,[s,zeros(length(s),1),zeros(length(s),1),l,zeros(length(s),1),zeros(length(s),1)]);
            replanePath_x = globalStates(:,1);
            replanePath_y = globalStates(:,2);
            replanStart_s = s_0;
            replanEnd_s = s_end;
            replanLSequcence = lSequcence;
            pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
                lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                changLaneEnd_s,changLaneStart_s,changLanePara,...
                replanStart_s,replanEnd_s,replanLSequcence);
            %速度规划
            [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
            if exitflag==1%速度规划ok
                %生成轨迹
                trajectoryType = 9;
            else
                %紧急制动轨迹
                trajectoryType = -9;
            end
        else %生成'replanPath'（当前重规划中）或者'currentLane'模式（当前不在重规划中）下的pathLine
            if isreplanPath == 1 || isreplanPath == 2 %重规划中
                replanePathFrenetState = global2frenet(refLine,[replanePath_x,replanePath_y,zeros(length(replanePath_x),4)]);
                index = find(replanePathFrenetState(:,1)>0,1);
                replanStart_s = replanePathFrenetState(index,1);
                replanEnd_s = replanePathFrenetState(end,1);
                replanLSequcence = replanePathFrenetState(index:end,4);
                pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    replanStart_s,replanEnd_s,replanLSequcence);
            else
                pathLine = piecewisePolynomial('currentLane',refLine,s_0,laneChangeDirection,...
                    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
                    changLaneEnd_s,changLaneStart_s,changLanePara,...
                    0,0,0);
            end
            %         plot(pathLine.breaks,ppval(pathLine,pathLine.breaks),'b.-' ); axis equal
            %         hold on
            %         plot(pathLine.breaks(1):1:pathLine.breaks(end),ppval(pathLine,pathLine.breaks(1):1:pathLine.breaks(end)),'r-' );
            %速度规划
            [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
            if isreplanPath == 1 || isreplanPath == 2 %重规划中
                if exitflag == 1%速度规划ok
                    %生成轨迹
                    trajectoryType = 10;
                else
                    %紧急制动轨迹
                    trajectoryType = -10;
                end
            else
                if exitflag == 1%速度规划ok
                    %生成轨迹
                    trajectoryType = 11;
                else
                    %紧急制动轨迹
                    trajectoryType = -11;
                end

            end
        end
    end
%     if ischanginglanes~=1 && isreplanPath == 1 %无需换道或换道不ok且重规划中 重规划过程中偏离了重规划路径需不需要重新规划？
%         pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
%             lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
%             changLaneEnd_s,changLaneStart_s,changLanePara,...
%             replanStart_s,replanEnd_s,replanLSequcence);
%         %速度规划
%         [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
%         if exitflag == 1%速度规划ok
%             %生成轨迹
%             trajectoryType = 9;
%         else
%             %紧急制动轨迹
%             trajectoryType = -9;
%         end
%         %重规划结束判断
%         if s_0 >= replanEnd_s
%             isreplanPath = 0;
%             replanStart_s = [];
%             replanEnd_s = [];
%             replanLSequcence = [];
%         end
%     elseif ischanginglanes~=1 && abs(l_0)>0.3 || abs(psi_0)>10 %无需换道或换道不ok且需重规划
%         %重规划原车道+ 速度规划---------------------------------
%         s_end = min(s_0+3*v_0+10,lineCurLane_s(end));
%         l_end = 0;
%         headingEnd = 0;
%         if ~isempty(lineleftLane_s)
%             offsetLeft2CurrentLane = abs(interp1(lineleftLane_s, lineleftLane_l, s_0+0.1));
%         else
%             offsetLeft2CurrentLane = 3.2;
%         end
%         if ~isempty(lineRightLane_s)
%             offsetRight2CurrentLane = abs(interp1(lineRightLane_s, lineRightLane_l, s_0+0.1));
%         else
%             offsetRight2CurrentLane = 3.2;
%         end
%         replan_sSequcence = lineCurLane_s;
%         replan_lRefSequcence = zeros(1,length(lineCurLane_s));
%         replan_rSequcence = abs(1./(lineCurLane_k+eps));
%         [lSequcence,replan_exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,v_0,replan_sSequcence,replan_lRefSequcence,replan_rSequcence,psi_0,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
%             CalibReplanPath,Parameter);
%         replanStart_s = s_0;
%         replanEnd_s = s_end;
%         replanLSequcence = lSequcence;
%         isreplanPath = 1;%1原车道
%         pathLine = piecewisePolynomial('replanPath',refLine,s_0,laneChangeDirection,...
%             lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
%             changLaneEnd_s,changLaneStart_s,changLanePara,...
%             replanStart_s,replanEnd_s,replanLSequcence);
%         %速度规划
%         [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
%         if exitflag==1%速度规划ok
%             %生成轨迹
%             trajectoryType = 10;
%         else
%             %紧急制动轨迹
%             trajectoryType = -10;
%         end
%     elseif  ischanginglanes~=1
%         pathLine = piecewisePolynomial('currentLane',refLine,s_0,laneChangeDirection,...
%             lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
%             changLaneEnd_s,changLaneStart_s,changLanePara,...
%             replanStart_s,replanEnd_s,replanLSequcence);
%         %         plot(pathLine.breaks,ppval(pathLine,pathLine.breaks),'b.-' ); axis equal
%         %         hold on
%         %         plot(pathLine.breaks(1):1:pathLine.breaks(end),ppval(pathLine,pathLine.breaks(1):1:pathLine.breaks(end)),'r-' );
%         %速度规划
%         [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars);
%         if exitflag == 1%速度规划ok
%             %生成轨迹
%             trajectoryType = 11;
%         else
%             %紧急制动轨迹
%             trajectoryType = -11;
%         end  
%     end
end
%轨迹生成
numOfPoint = length(sOptSequence);
trajFrenet = zeros(numOfPoint,6);
trajFrenet(:,1) = sOptSequence';
trajFrenet(:,2) = vOptSequence';
trajFrenet(:,3) = aOptSequence';
trajGlobal = frenet2global(refPath,trajFrenet);
traj.x = trajGlobal(:,1);
traj.y = trajGlobal(:,2);
traj.theta = trajGlobal(:,3);
traj.k = trajGlobal(:,4);
traj.speed = trajGlobal(:,5);
traj.acce = trajGlobal(:,6);
%GlobVars
GlobVars.urbanPlanner.ischanginglanes = ischanginglanes;
GlobVars.urbanPlanner.isreplanPath = isreplanPath;
GlobVars.urbanPlanner.changLaneStart_s = changLaneStart_s;
GlobVars.urbanPlanner.changLaneEnd_s = changLaneEnd_s;
GlobVars.urbanPlanner.changLanePara = changLanePara;
% GlobVars.urbanPlanner.replanStart_s = replanStart_s;
% GlobVars.urbanPlanner.replanEnd_s = replanEnd_s;
% GlobVars.urbanPlanner.replanLSequcence = replanLSequcence;
GlobVars.urbanPlanner.replanePath_x = replanePath_x;
GlobVars.urbanPlanner.replanePath_y = replanePath_y;
GlobVars.urbanPlanner.curTargetLaneIndex = curTargetLaneIndex;
GlobVars.urbanPlanner.laneChangeDirection = laneChangeDirection;
%画图
%SL
figure(2);
if isempty(gcf().Children)
    title('Planning S-L-Graph')
    xlabel('s(meter)')
    ylabel('l(meter)')
else
    cla(gcf().Children)
end
hold on 
P = [(pathLine.breaks(1):1:pathLine.breaks(end))',(ppval(pathLine,pathLine.breaks(1):1:pathLine.breaks(end)))'];
plot(P(:,1),P(:,2),'r.-' );
xlim([s_0-5 s_0+60]);
ylim([-5 5]);
polyout2 = polybuffer(P,'lines',pathBuffer+0.5*w_veh);
plot(polyout2)
plot(s_0,l_0,'bo');
for index_obstacle = 1:ObstaclesFrenetState.obstacleNum
    plot(ObstaclesFrenetState.obstacles(index_obstacle).s,ObstaclesFrenetState.obstacles(index_obstacle).l,'y.',...
        ObstaclesFrenetState.obstacles(index_obstacle).s(1),ObstaclesFrenetState.obstacles(index_obstacle).l(1),'co')
end
end




