function [Decision,GlobVars]=Decider(PlannerLevel,BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,StopSignInfo,...
    AvoFailVehInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,GlosaActive,AEBActive,TargetGear,a_soll_ACC,...
    a_soll_SpeedPlanAvoidPedestrian,a_soll_TrafficLightActive,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_sollTurnAround2Decider,...
    a_soll_Fail,TargetLaneIndex,BackupTargetLaneIndex,d_veh2stopline_ped,GlobVars,CalibrationVars,Parameters)
%globalVariable-------------------------------------------------------------------------------------------------------------------------------------
CountLaneChange=GlobVars.Decider.countLaneChangeDecider;
CurrentTargetLaneIndex=GlobVars.Decider.currentTargetLaneIndexDecider;
dec_start=GlobVars.Decider.dec_start;
dir_start=GlobVars.Decider.dir_start;
wait_pullover=GlobVars.Decider.wait_pullover;
distBehindGoal=GlobVars.Decider.distBehindGoal;
dec_follow=GlobVars.Decider.dec_follow;
%CalibrationVars------------------------------------------------------------------------------------------------------------------------------------
a_bre=CalibrationVars.Decider.a_bre;%-3;%m/s^2
a_bre_com=CalibrationVars.Decider.a_bre_com;%-1.5;%m/s^2
idle_speed=CalibrationVars.Decider.idle_speed;%7;%km/h
glosaVMin=CalibrationVars.Decider.glosaVMin;%20;%km/h
dist_wait2pilot=CalibrationVars.Decider.dist_wait2pilot;%10;%m
dist_wait2veh=CalibrationVars.Decider.dist_wait2veh;%15;%m
GlosaAverageIndex=CalibrationVars.Decider.glosaAverageIndex;%0.8
jerkLimit=CalibrationVars.UrbanPlanner.jerkLimit;%2
d_gap2waitingArea = CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_gap2waitingArea;
if PlannerLevel==2
    dist_wait= dist_wait2veh;
else %PlannerLevel==3
    dist_wait= dist_wait2pilot;
end
%入参-----------------------------------------------------------------------------------------------------------------------------------------------
SampleTime=BasicsInfo.sampleTime;
l_veh=Parameters.l_veh;
d_veh2goal=BasicsInfo.d_veh2goal;
d_wait=CalibrationVars.ACC.d_wait;%4
tau_v = CalibrationVars.ACC.tau_v;
% v_max_SpeedPlanAvoidVehicle=CalibrationVars.SpeedPlanAvoidVehicle.v_max;%40/3.6;
v_max = BasicsInfo.v_max;
speed = ChassisInfo.speed;
wait_ped = GlobVars.SpeedPlanAvoidPedestrian.wait_ped;
d_veh2cross = AvoPedInfo.d_veh2cross;
wait_TrafficLight=GlobVars.SpeedPlanTrafficLight.wait_TrafficLight;
wait_AvoidVehicle=GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle;
wait_avoidOncomingVehicle=GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;
d_veh2Rampstopline = AvoMainRoVehInfo.d_veh2stopline;
d_veh2waitingArea = AvoOncomingVehInfo.d_veh2waitingArea-d_gap2waitingArea;
d_veh2Intstopline = TrafficLightInfo.d_veh2stopline;
d_veh2Signstopline = StopSignInfo.d_veh2stopline;
CurrentLaneFrontDis = BasicsInfo.currentLaneFrontDis;
CurrentLaneFrontVel = BasicsInfo.currentLaneFrontVel;
CurrentLaneFrontLen = BasicsInfo.currentLaneFrontLen;
FailLaneindex = AvoFailVehInfo.failLaneindex;% 故障车所在车道序号,数组大小5
FailLaneFrontDis = AvoFailVehInfo.failLaneFrontDis;
FailLaneFrontVel = AvoFailVehInfo.failLaneFrontVel;
FailLaneFrontLen = AvoFailVehInfo.failLaneFrontLen;
WidthOfLanes = BasicsInfo.widthOfLanes;
CurrentLaneIndex = BasicsInfo.currentLaneIndex;
%-----------------------------------------------------
FailIndex = find(FailLaneindex == CurrentLaneIndex);
if ~isempty(FailIndex)
    if length(FailIndex)>1
        CountLaneFailDis = FailLaneFrontDis(FailIndex);
        CountLaneFailVel = FailLaneFrontVel(FailIndex);
        CountLaneFailLen = FailLaneFrontLen(FailIndex);
        [CountLaneFailDis,I] = sort(CountLaneFailDis);
        CountLaneFailVel = CountLaneFailVel(I);
        CountLaneFailLen = CountLaneFailLen(I);
    else
        CountLaneFailDis = FailLaneFrontDis(FailIndex);
        CountLaneFailVel = FailLaneFrontVel(FailIndex);
        CountLaneFailLen = FailLaneFrontLen(FailIndex);
    end
    if CountLaneFailDis(1) < CurrentLaneFrontDis
        CurrentLaneFrontDis = CountLaneFailDis(1);
        CurrentLaneFrontVel = CountLaneFailVel(1);
        CurrentLaneFrontLen = CountLaneFailLen(1);
    end
end
% TargetLaneIndex = BasicsInfo.TargetLaneIndex;%目标车道取自避让故障概车函数
d_veh2int = LaneChangeInfo.d_veh2int;
LeftLaneBehindDis = LaneChangeInfo.leftLaneBehindDis;
LeftLaneBehindVel = LaneChangeInfo.leftLaneBehindVel;
LeftLaneFrontDis = LaneChangeInfo.leftLaneFrontDis;
LeftLaneFrontVel = LaneChangeInfo.leftLaneFrontVel;
LeftLaneFrontLen = LaneChangeInfo.leftLaneFrontLen;
RightLaneBehindDis = LaneChangeInfo.rightLaneBehindDis;
RightLaneBehindVel = LaneChangeInfo.rightLaneBehindVel;
RightLaneFrontDis = LaneChangeInfo.rightLaneFrontDis;
RightLaneFrontVel = LaneChangeInfo.rightLaneFrontVel;
RightLaneFrontLen = LaneChangeInfo.rightLaneFrontLen;
LeftLaneBehindLen = LaneChangeInfo.leftLaneBehindLen;
RightLaneBehindLen = LaneChangeInfo.rightLaneBehindLen;
% Environmental car -------------------------------------------------------------------------------------------------------------------------------
% CurrentLaneFrontDis = CurrentLaneFrontDis-CurrentLaneFrontLen;
% RightLaneFrontDis=RightLaneFrontDis-RightLaneFrontLen;
% LeftLaneFrontDis=LeftLaneFrontDis-LeftLaneFrontLen;

d_veh2goal=d_veh2goal-0.5*Parameters.l_veh;%车中心转车头
CurrentLaneFrontDis=CurrentLaneFrontDis-0.5*(CurrentLaneFrontLen+Parameters.l_veh);%车头到前车车尾距离
%避让对向车
d_veh2waitingArea=d_veh2waitingArea-0.5*Parameters.l_veh;%车中心距离转为车头距离
%换道入参：
d_veh2int=d_veh2int-0.5*Parameters.l_veh;%车中心距离转为车头距离
RightLaneFrontDis=RightLaneFrontDis-0.5*(RightLaneFrontLen+Parameters.l_veh);%车头到车尾距离
LeftLaneFrontDis=LeftLaneFrontDis-0.5*(LeftLaneFrontLen+Parameters.l_veh);%车头到车尾距离
RightLaneBehindDis = RightLaneBehindDis+0.5*(Parameters.l_veh-RightLaneBehindLen);%车头到车头距离
LeftLaneBehindDis = LeftLaneBehindDis+0.5*(Parameters.l_veh-LeftLaneBehindLen);%车头到车头距离
%避让同向车入参：车中心距离转为车头距离
d_veh2Rampstopline=d_veh2Rampstopline-0.5*Parameters.l_veh;
%停车让行停止线距离
% d_veh2Signstopline=d_veh2Signstopline-0.5*Parameters.l_veh;%%车中心距离转为车头距离
%信号灯通行
d_veh2Intstopline=d_veh2Intstopline-0.5*Parameters.l_veh;%%车中心距离转为车头距离
%行人
d_veh2cross=d_veh2cross-0.5*Parameters.l_veh;%%车中心距离转为车头距离

%--------------------------------------------------------------------------------------------------------------------------------------------------
w_lane_left=0.5*WidthOfLanes(max(CurrentLaneIndex-1,1))+0.5*WidthOfLanes(CurrentLaneIndex);
w_lane_right=0.5*WidthOfLanes(min(CurrentLaneIndex+1,6))+0.5*WidthOfLanes(CurrentLaneIndex);
%初值
TargetVelocity=double(-20);
TargetSpeed=double(-20);
SlowDown=int16(0);
Wait=int16(0);
decision_states=int16(0);
%初始化
Decision.AEBactive=AEBActive;
Decision.TargetGear=TargetGear;
Decision.states=int16(0);
Decision.LaneChange=int16(0);
Decision.SlowDown=int16(0);
Decision.TargetSpeed=double(0);
Decision.Wait=int16(0);
Decision.WaitDistance=double(200);
Decision.Start=int16(0);
Decision.a_soll=double(0);
Decision.LaneChange=int16(0);
Decision.PedestrianState=int16(0);
Decision.TrafficLightState=int16(0);
Decision.VehicleCrossingState=int16(0);
Decision.VehicleOncomingState=int16(0);
Decision.StopSignState=int16(0);
Decision.FollowState=int16(0);
Decision.PullOverState=int16(0);
%
if GlobVars.TrajPlanTurnAround.typeOfTurnAround==2
    decision_states=int16(3);
end
if AEBActive~=0
    decision_states=int16(2);
end
%LaneChangeDecision--------------------------------------------------------------------------------------------------------------------------------
if TargetLaneIndex<=CurrentLaneIndex
    TargetLaneBehindDis=LeftLaneBehindDis;
    TargetLaneBehindVel=LeftLaneBehindVel;
    TargetLaneFrontDis=LeftLaneFrontDis;
    TargetLaneFrontVel=LeftLaneFrontVel;
else
    TargetLaneBehindDis=RightLaneBehindDis;
    TargetLaneBehindVel=RightLaneBehindVel;
    TargetLaneFrontDis=RightLaneFrontDis;
    TargetLaneFrontVel=RightLaneFrontVel;
end
if BackupTargetLaneIndex~=-1
    s_d=RightLaneBehindDis;
    v_d=RightLaneBehindVel;
    s_e=RightLaneFrontDis;
    v_e=RightLaneFrontVel;
    BackupTargetLaneIndex=int16(min([CurrentLaneIndex+1 BackupTargetLaneIndex]));
    BackupTargetLaneIndex=int16(max([CurrentLaneIndex-1 BackupTargetLaneIndex]));
else
    s_d=-200;
    v_d=-20;
    s_e=200;
    v_e=20;
end

% TargetLaneIndex=TargetLaneIndex-1;
% CurrentLaneIndex=CurrentLaneIndex-1;
s_a=CurrentLaneFrontDis;
v_a=CurrentLaneFrontVel;
s_b=TargetLaneBehindDis;
v_b=TargetLaneBehindVel;
s_c=TargetLaneFrontDis;
v_c=TargetLaneFrontVel;
% v_max_int=CalibrationVars.TrajPlanLaneChange.v_max_int;%30/3.6;
indexAfterLaneChangeDis2Int=CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int;%1;
% t_permit=CalibrationVars.TrajPlanLaneChange.t_permit;%3;
t_re=CalibrationVars.TrajPlanLaneChange.t_re;%0.5;
index_accel=CalibrationVars.TrajPlanLaneChange.index_accel;%0.5;
a_max_comfort=CalibrationVars.TrajPlanLaneChange.a_max_comfort;%1;
a_min=CalibrationVars.TrajPlanLaneChange.a_min;%-3.5;
a_max=CalibrationVars.TrajPlanLaneChange.a_max;%2.5;
a_min_comfort=CalibrationVars.TrajPlanLaneChange.a_min_comfort;%-1;
% a_soll=100;
% t_lc=max([2-0.05*(speed-10) 1.7]);
t_lc=max([2-0.04*(speed-15) 2]);
% t_lc=min([t_lc 2.3]);
t_lc=min([t_lc 2.5]);
t_lc=ceil(t_lc/0.1)*0.1;
% traj=zeros([6 120]);
% l_veh=Parameters.l_veh;
% w_veh=1.8;
TargetLaneIndex=min([CurrentLaneIndex+1 TargetLaneIndex]);
TargetLaneIndex=max([CurrentLaneIndex-1 TargetLaneIndex]);
% S_traj=zeros(1,4/0.05);
% X_traj=zeros(1,4/0.05);
% V_traj=zeros(1,4/0.05);

% wait=0;
% if isempty(LanesWithFail)==0
%     for i=LanesWithFail
%         if CurrentLaneIndex==i
%             wait=-1;
%         end
%     end
% end

% Lane change decision
S_0=0;
V_0=speed;
% S_end=0;%2020324,编译c报错增加初始值
% V_end=0;
CountLaneChange=int16(0);
if CurrentTargetLaneIndex==CurrentLaneIndex
    CountLaneChange=int16(0);
end
if CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex
%     if TargetLaneIndex<=CurrentLaneIndex
%         w_lane=w_lane_left;
%     else
%         w_lane=w_lane_right;
%     end
    w_lane=0.5*WidthOfLanes(CurrentLaneIndex)+0.5*WidthOfLanes(TargetLaneIndex);%20220328

    if speed<5
        S_end=max([10 t_lc*speed]);
        fun_t=@(t)(speed*t+0.5*2*t.^2);
        [t_lc_strich,~,~] = fzero(@(t)fun_t(t)-(S_end.^2+w_lane.^2).^0.5,[1 4]);
        t_lc=max([t_lc t_lc_strich]);
        t_lc=0.1*round(t_lc/0.1);
        S_min_dyn=max([(max([w_lane (0-V_0.^2)/(2*a_min)]).^2-w_lane.^2).^0.5 (max([w_lane S_0+V_0*t_lc+0.5*a_min*t_lc*t_lc]).^2-w_lane.^2).^0.5]);
        
        index_accel_strich=max([ACC(v_max,v_c,s_c-s_b,v_b,0,CalibrationVars)/a_max_comfort 0.5]);
        V_c_end=max([0 v_c+(index_accel*a_min_comfort)*t_lc]);
        S_c_end=s_c+0.5*(V_c_end+v_c)*t_lc;
        V_b_end=max([0 v_b+(index_accel_strich*a_max_comfort)*t_lc]);
        S_b_end=s_b+0.5*(V_b_end+v_b)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        % S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc;
        % S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % V_c_end=v_c+(index_accel*a_min_comfort)*t_lc;
        % V_b_end=v_b+(index_accel_strich*a_max_comfort)*t_lc;

        if (-s_b>v_b*t_lc) && ...,
                (s_c<=V_0*t_lc)
            %                 b车不存在 c车存在
            % V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif ( -s_b<=v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车存在 c车不存在
            % V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif (-s_b>v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车不存在 c车不存在
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        else
            %                 b车存在 c车存在
            % V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        end
        prereq1=(S_c_end-S_b_end-l_veh>V_b_end*t_re+(V_c_end.^2-V_b_end.^2)/(2*a_min));
%         prereq2=(S_a_end>0.5*(S_0+S_end));
        prereq3=(V_end>V_0+a_min*t_lc);
        prereq4=(V_end<V_0+a_max*t_lc);
        prereq5=(S_max>S_end&&S_end>S_min);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh || d_veh2goal<d_veh2int; % 距离路口过近时不允许换道
        prereq7=(s_b<=-l_veh);
        prereq8=(s_c>=0);
        prereq9=(S_a_mid>0.5*S_end);
        prereq10=(s_a>=0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    else
        index_accel_strich=max([ACC(v_max,v_c,s_c-s_b,v_b,0,CalibrationVars)/a_max_comfort 0.5]);
        V_c_end=max([0 v_c+(index_accel*a_min_comfort)*t_lc]);
        S_c_end=s_c+0.5*(V_c_end+v_c)*t_lc;
        V_b_end=max([0 v_b+(index_accel_strich*a_max_comfort)*t_lc]);
        S_b_end=s_b+0.5*(V_b_end+v_b)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;

        % S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc;
        % S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % V_c_end=v_c+(index_accel*a_min_comfort)*t_lc;
        % V_b_end=v_b+(index_accel*index_accel_strich)*t_lc;
        if (-s_b>v_b*t_lc) && ...,
                (s_c<=V_0*(t_lc+t_re))
            %                 b车不存在 c车存在
            V_end=min([0.5*(V_0+V_c_end) V_0]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif ( -s_b<=v_b*t_lc) && ...,
                ( s_c>V_0*(t_lc+t_re))
            %                 b车存在 c车不存在
            V_end=min([0.5*(V_0+V_b_end) V_0]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif (-s_b>v_b*t_lc) && ...,
                ( s_c>V_0*(t_lc+t_re))
            %                 b车不存在 c车不存在
            S_end=V_0*t_lc;
            V_end=V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
        else
            %                 b车存在 c车存在
            V_end=min([V_0 0.5*(V_b_end+V_c_end)]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        end
        % 参见TrajPlanLaneChange_S_max_withAccel.bmp
        S_max_withAccel=t_lc*min([V_end V_0])+0.5*a_max_comfort*t_lc*t_lc-((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 
        S_min_withAccel=t_lc*max([V_end V_0])-0.5*a_max_comfort*t_lc*t_lc+((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移
        prereq1=(S_c_end-S_b_end-l_veh>V_b_end*t_re+(V_c_end.^2-V_b_end.^2)/(2*a_min));
%         prereq2=(S_a_end>0.5*(S_0+S_end));
        prereq3=(V_end>=V_0+a_min_comfort*t_lc);
        prereq4=(V_end<=V_0+a_max_comfort*t_lc);
        prereq5=(S_max>=S_min&&S_end<=S_max_withAccel&&S_end>=S_min_withAccel);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh || d_veh2goal<d_veh2int; % 距离路口过近时不允许换道
        % prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道
        prereq7=(s_b<=-l_veh);
        prereq8=(s_c>=0);
        prereq9=(S_a_mid>0.5*S_end);
        prereq10=(s_a>=0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    end
end
if  CurrentTargetLaneIndex~=TargetLaneIndex&&CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && -1~=BackupTargetLaneIndex
    if BackupTargetLaneIndex<=CurrentLaneIndex
        w_lane=w_lane_left;
    else
        w_lane=w_lane_right;
    end
    if speed<5
        S_end=max([10 t_lc*speed]);
        fun_t=@(t)(speed*t+0.5*2*t.^2);
        [t_lc_strich,~,~] = fzero(@(t)fun_t(t)-(S_end.^2+w_lane.^2).^0.5,[1 4]);
        t_lc=max([t_lc t_lc_strich]);
        t_lc=0.1*round(t_lc/0.1);
        S_min_dyn=max([(max([w_lane (0-V_0.^2)/(2*a_min)]).^2-w_lane.^2).^0.5 (max([w_lane S_0+V_0*t_lc+0.5*a_min*t_lc*t_lc]).^2-w_lane.^2).^0.5]);

        index_accel_strich=max([ACC(v_max,v_e,s_e-s_d,v_d,0,CalibrationVars)/a_max_comfort 0.5]);
        V_e_end=max([0 v_e+(index_accel*a_min_comfort)*t_lc]);
        S_e_end=s_e+0.5*(V_e_end+v_e)*t_lc;
        V_d_end=max([0 v_d+(index_accel_strich*a_max_comfort)*t_lc]);
        S_d_end=s_d+0.5*(V_d_end+v_d)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        if (-s_d>v_d*t_lc) && ...,
                (s_e<=V_0*t_lc)
            %                 b车不存在 c车存在
            % V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_e_end-t_re*V_end S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif ( -s_d<=v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车存在 c车不存在
            % V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_e_end-t_re*V_end S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif (-s_d>v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车不存在 c车不存在
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_e_end-t_re*V_end S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        else
            %                 b车存在 c车存在
            % V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
%             S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
            S_max=min([S_e_end-t_re*V_end S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        end
        prereq1=(S_e_end-S_d_end-l_veh>V_d_end*t_re+(V_e_end.^2-V_d_end.^2)/(2*a_min));
%         prereq2=(S_a_end>0.5*(S_0+S_end));
        prereq3=(V_end>V_0+a_min*t_lc);
        prereq4=(V_end<V_0+a_max*t_lc);
        prereq5=(S_max>S_end&&S_end>S_min);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh || d_veh2goal<d_veh2int; % 距离路口过近时不允许换道
        prereq7=(s_d<=-l_veh);
        prereq8=(s_e>=0);
        prereq9=(S_a_mid>0.5*S_end);
        prereq10=(s_a>=0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            TargetLaneIndex=BackupTargetLaneIndex;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    else
        index_accel_strich=max([ACC(v_max,v_e,s_e-s_d,v_d,0,CalibrationVars)/a_max_comfort 0.5]);
        V_e_end=max([0 v_e+(index_accel*a_min_comfort)*t_lc]);
        S_e_end=s_e+0.5*(V_e_end+v_e)*t_lc;
        V_d_end=max([0 v_d+(index_accel_strich*a_max_comfort)*t_lc]);
        S_d_end=s_d+0.5*(V_d_end+v_d)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        if (-s_d>v_d*t_lc) && ...,
                (s_e<=V_0*t_lc)
            %                 b车不存在 c车存在
            V_end=min([0.5*(V_0+V_e_end) V_0]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif ( -s_d<=v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车存在 c车不存在
            V_end=min([0.5*(V_0+V_d_end) V_0]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif (-s_d>v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车不存在 c车不存在
            S_end=V_0*t_lc;
            V_end=V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
        else
            %                 b车存在 c车存在
            V_end=min([V_0 0.5*(V_d_end+V_e_end)]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
%             S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min) S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        end
        S_max_withAccel=t_lc*min([V_end V_0])+0.5*a_max_comfort*t_lc*t_lc-((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移
        S_min_withAccel=t_lc*max([V_end V_0])-0.5*a_max_comfort*t_lc*t_lc+((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移
        prereq1=(S_e_end-S_d_end-l_veh>V_d_end*t_re+(V_e_end.^2-V_d_end.^2)/(2*a_min));
%         prereq2=(S_a_end>0.5*(S_0+S_end));
        prereq3=(V_end>=V_0+a_min_comfort*t_lc);
        prereq4=(V_end<=V_0+a_max_comfort*t_lc);
        prereq5=(S_max>=S_min&&S_end<=S_max_withAccel&&S_end>=S_min_withAccel);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh || d_veh2goal<d_veh2int; % 距离路口过近时不允许换道
        % prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道
        prereq7=(s_d<=-l_veh);
        prereq8=(s_e>=0);
        prereq9=(S_a_mid>0.5*S_end);
        prereq10=(s_a>=0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            TargetLaneIndex=BackupTargetLaneIndex;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    end
end
if LaneChangeActive
    LaneChangeDecision=CurrentTargetLaneIndex-CurrentLaneIndex;
    if CountLaneChange>0&&LaneChangeDecision==-1%左换道
        Decision.LaneChange=int16(1); 
    elseif CountLaneChange>0&&LaneChangeDecision==1%右换道
        Decision.LaneChange=int16(2);
    else
        Decision.LaneChange=int16(0);
%         GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0);
    end
else
    Decision.LaneChange=int16(0);
%     GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0);
    CountLaneChange=int16(0);
end
%-------------------------------------------------------------------------------------------------------------------------------------------------------
%% status
if PedestrianActive
    Decision.PedestrianState=int16(1);
else
    Decision.PedestrianState=int16(0);
end
if TrafficLightActive
    Decision.TrafficLightState=int16(1); 
else
    Decision.TrafficLightState=int16(0);
end
if VehicleCrossingActive
    Decision.VehicleCrossingState=int16(1);
else
    Decision.VehicleCrossingState=int16(0);
end
if VehicleOncomingActive
    Decision.VehicleOncomingState=int16(1);
else
    Decision.VehicleOncomingState=int16(0);
end
if CurrentLaneFrontDis<195&&CurrentLaneFrontVel<20
    Decision.FollowState=int16(1);
else
    Decision.FollowState=int16(0);
end
if GlobVars.SpeedPlanStopSign.wait_stopsign==1
    Decision.StopSignState=int16(1);
else
    Decision.StopSignState=int16(0);
end
if d_veh2goal<=60%靠边停车
    Decision.PullOverState=int16(1);
else
    Decision.PullOverState=int16(0);
end
if GlobVars.TrajPlanTurnAround.turnAroundActive==1
    Decision.TurnAroundState=int16(1);
else
    Decision.TurnAroundState=int16(0);
end
%% wait
% CalibrationVars.Decider.GlosaAdp=2;
% CalibrationVars.Decider.mrg=4;
% CalibrationVars.Decider.desRate=0.75;
% CalibrationVars.Decider.dIntxn=10;
% CalibrationVars.Decider.dMin=2;
% TrafficLightInfo.Phase=zeros(1,10);
if GlosaActive==1 && TrafficLightActive==1 && d_veh2Intstopline>0
    [~,vgMin,vgMax]=scen_glosa(d_veh2Intstopline, speed, TrafficLightInfo.phase, v_max, glosaVMin/3.6, CalibrationVars.Decider.glosaAdp,CalibrationVars.Decider.dec,...,
        CalibrationVars.Decider.mrg,CalibrationVars.Decider.desRate, CalibrationVars.Decider.dIntxn, CalibrationVars.Decider.dMin);
    if vgMin==-1
        a_soll_TrafficLightActive=ACC(v_max,0,d_veh2Intstopline+CalibrationVars.ACC.d_wait-0.5,speed,1,CalibrationVars);
    else
        if speed>vgMax*GlosaAverageIndex+vgMin*(1-GlosaAverageIndex)
            a_soll_TrafficLightActive=-CalibrationVars.Decider.glosaAdp;
        elseif speed<vgMin*GlosaAverageIndex+vgMax*(1-GlosaAverageIndex)
            a_soll_TrafficLightActive=CalibrationVars.Decider.glosaAdp;
        else
            a_soll_TrafficLightActive=0;
        end
    end
else
    vgMin=0;%Variable 'vgMin' is not fully defined on some execution paths.
end
wait_matrix=zeros(1,8)+200;
% if CurrentLaneFrontVel<=0.1 && CurrentLaneFrontDis<=dist_wait+l_veh+5
%     wait_matrix(6)=CurrentLaneFrontDis-l_veh-5;
% end
if CurrentLaneFrontVel<=0.5 && CurrentLaneFrontDis<=dist_wait+d_wait%CurrentLaneFrontDis为前车车尾距离
    wait_matrix(7)=CurrentLaneFrontDis-d_wait;
end
if Decision.TurnAroundState==1&&(GlobVars.TrajPlanTurnAround.wait_turnAround==1||wait_TrafficLight==1)&&GlobVars.TrajPlanTurnAround.posCircle(1)-BasicsInfo.pos_s<=dist_wait
    wait_matrix(5)=GlobVars.TrajPlanTurnAround.posCircle(1)-BasicsInfo.pos_s;%掉头激活且掉头停止线小于停车距离,掉头wait和信号灯wait任意=1
end
if wait_AvoidVehicle==1 && d_veh2Rampstopline<=dist_wait
    wait_matrix(2)=d_veh2Rampstopline;
end
if wait_avoidOncomingVehicle==1 && d_veh2waitingArea<=dist_wait
    wait_matrix(3)=d_veh2waitingArea;
end
% if wait_ped==1 && d_veh2cross<=dist_wait
%     wait_matrix(1)=d_veh2cross;
% end
if wait_ped==1 && d_veh2stopline_ped<=dist_wait
    wait_matrix(1)=d_veh2stopline_ped;
end
if GlosaActive==1 && TrafficLightActive==1 && d_veh2Intstopline>0
    if vgMin==-1 && d_veh2Intstopline<=dist_wait
        wait_matrix(4)=d_veh2Intstopline;
    end
else
    if wait_TrafficLight==1 && d_veh2Intstopline<=dist_wait
        wait_matrix(4)=d_veh2Intstopline;
    end
end
if GlobVars.SpeedPlanStopSign.wait_stopsign==1&&d_veh2Signstopline<=dist_wait&&d_veh2Signstopline>=0
    wait_matrix(6)=d_veh2Signstopline;
end
% if d_veh2goal<=dist_wait%靠边停车
%     wait_matrix(7)=d_veh2goal;
% end
% 靠边停车子功能wait状态的切换及靠边停车位置的计算
if CurrentLaneIndex==TargetLaneIndex && BasicsInfo.d_veh2goal<60 && wait_pullover==0
    if speed.^2/(2*2)<15
        wait_pullover=int16(1);
        distBehindGoal=max(0,speed.^2/(2*2)-BasicsInfo.d_veh2goal); % distBehindGoal为全局变量，初值为0
    end
end
% 靠边停车距离的计算
if wait_pullover==1
    dist_pullover=BasicsInfo.d_veh2goal+distBehindGoal;
    wait_matrix(8)=dist_pullover;
end
if min(wait_matrix)<=dist_wait
    WaitDistance=min(wait_matrix);
    for a_soll_index=int16(1):length(wait_matrix)
        if wait_matrix(a_soll_index)==WaitDistance
            Wait=int16(a_soll_index);
            break
        end
    end
else
    Wait=int16(0);
    WaitDistance=200;
end
if Wait==2&&d_veh2Signstopline<=dist_wait&&d_veh2Signstopline>=0%有停止让行标志
    Wait=int16(6);
end
%% slowdown
if v_max-speed<0%限速加速度
    accel_speedlimit=max([-2.5 (v_max-speed)/tau_v]);
else
    accel_speedlimit=100;
end
if GlobVars.SpeedPlanStopSign.wait_stopsign==1%停车让行加速度
    a_soll_StopSign=ACC(v_max,0,max([0 StopSignInfo.d_veh2stopline+CalibrationVars.ACC.d_wait]),speed,0,CalibrationVars);
else
    a_soll_StopSign=100;
end
if BasicsInfo.d_veh2goal<60%靠边停车
    if CurrentLaneIndex~=TargetLaneIndex%未在目标车道较晚减速
        if BasicsInfo.d_veh2goal<((CalibrationVars.TrajPlanLaneChange.v_max_int.^2-v_max.^2)/(2*(-1.5))+CalibrationVars.TrajPlanLaneChange.v_max_int*CalibrationVars.TrajPlanLaneChange.t_permit)
            a_soll_pullover=ACC(30/3.6,20,200,speed,0,CalibrationVars);
        else
            a_soll_pullover=100;
        end
    else
        a_soll_pullover=ACC(v_max,0,BasicsInfo.d_veh2goal+CalibrationVars.ACC.d_wait,speed,1,CalibrationVars);
        if BasicsInfo.d_veh2goal<=CalibrationVars.Decider.d_veh2endpoint&&speed<0.2
            decision_states=int16(1);
        end
    end
else
    a_soll_pullover=100;
end
a_soll_ACC=min(a_soll_Fail,a_soll_ACC);
% a_soll_matrix=[a_soll_SpeedPlanAvoidPedestrian,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_soll_TrafficLightActive,a_soll_StopSign,a_soll_ACC,a_soll_veh2goal,accel_speedlimit];
a_soll_matrix=[a_soll_SpeedPlanAvoidPedestrian,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_soll_TrafficLightActive,a_sollTurnAround2Decider,a_soll_StopSign,a_soll_ACC,a_soll_pullover,accel_speedlimit];
a_soll=min(a_soll_matrix);
a_sollTargetSpeed=a_soll;
if speed<=0
  a_soll=max(0,a_soll);  
end
dlimit=(CurrentLaneFrontVel^2-speed^2)*0.5/a_bre;%跟车安全距离
if dec_follow==0%跟车减速指示的决策
    if (CurrentLaneFrontVel^2-speed^2)*0.5/a_bre+d_wait>CurrentLaneFrontDis
        dec_follow=int16(1);
    end
else
    if (CurrentLaneFrontVel^2-speed^2)*0.5/a_bre_com+d_wait<CurrentLaneFrontDis
        dec_follow=int16(0);
    end
end
if PlannerLevel==2
    for a_soll_index=length(a_soll_matrix):int16(-1):int16(1)%从后往前查
        if a_soll_matrix(a_soll_index)==a_soll
            break
        end
    end
    SlowDown=int16(0);
    if a_soll_index==1
        if a_soll<=-0.2||wait_ped==1
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==2
        if a_soll<=-0.2||wait_AvoidVehicle==1
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==3
        if a_soll<=-0.2||wait_avoidOncomingVehicle==1
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==4
        if a_soll<=-0.2&&wait_TrafficLight==1
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==5
        if a_soll<=-0.2
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==6
        if a_soll<=-0.2
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==7
        if dec_follow==1 || a_soll_Fail < 0
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==8
        if a_soll<=-0.2
            SlowDown=a_soll_index;
        end
    elseif a_soll_index==9
        if a_soll<=-0.2
            SlowDown=a_soll_index;
        end
    end
    %TargetSpeed
    if Wait>0
        TargetSpeed=-20;
        SlowDown=int16(0);
        GlobVars.Decider.a_soll_pre=100; % GlobVars.Decider.a_soll_pre的初始值为100
    elseif PedestrianActive||TrafficLightActive||VehicleCrossingActive||VehicleOncomingActive|| Decision.TurnAroundState||SlowDown==7||Decision.StopSignState||Decision.PullOverState%跟车减速提示时，停车让行时，靠边停车时
%         TargetSpeed=(speed+a_soll*SampleTime);
        if GlobVars.Decider.a_soll_pre~=100
            if a_soll>-2
                a_sollTargetSpeed=median([GlobVars.Decider.a_soll_pre+jerkLimit*SampleTime,a_soll,GlobVars.Decider.a_soll_pre-jerkLimit*SampleTime]);
            else
                a_sollTargetSpeed=median([GlobVars.Decider.a_soll_pre+jerkLimit*SampleTime,a_soll,GlobVars.Decider.a_soll_pre-2.5*jerkLimit*SampleTime]);
            end
        else
            a_sollTargetSpeed=a_soll;
        end
        TargetSpeed=max(0,speed+a_sollTargetSpeed*SampleTime);
        GlobVars.Decider.a_soll_pre=a_sollTargetSpeed; % GlobVars.Decider.a_soll_pre只有在下发目标速度时才更新
    else
        TargetSpeed=v_max;
        GlobVars.Decider.a_soll_pre=100;
    end
elseif PlannerLevel==3%驾驶员
    TargetVelocity=sqrt(-dist_wait*2*a_bre_com);
    dislevel1=((TargetVelocity)^2-(30/3.6)^2)*0.5/a_bre_com;
    dislevel2=((30/3.6)^2-(40/3.6)^2)*0.5/a_bre_com;
    dislevel3=((40/3.6)^2-(50/3.6)^2)*0.5/a_bre_com;
    
    if a_soll<-0.2&&Wait==0%wait时速度默认值
            for a_soll_index=length(a_soll_matrix):int16(-1):int16(1)%从后往前查
                if a_soll_matrix(a_soll_index)==a_soll
                    SlowDown=a_soll_index;
                    break
                end
            end
        if SlowDown==7 %followcar
            if CurrentLaneFrontVel<=0.1&&CurrentLaneFrontDis>=dist_wait
                if CurrentLaneFrontDis<=dislevel1+dist_wait
                    TargetVelocity=round(TargetVelocity*3.6);%km/h
                elseif CurrentLaneFrontDis<=dislevel1+dislevel2+dist_wait
                    TargetVelocity=30;%km/h
                elseif CurrentLaneFrontDis<=dislevel1+dislevel2+dislevel3+dist_wait
                    TargetVelocity=40;%km/h
                else
                    TargetVelocity=-20;
                end
            else
                TargetVelocity=CurrentLaneFrontVel*3.6;%km/h
            end
        elseif SlowDown==9 %Speedlimit
            if speed>=v_max*1.1  %超过限速%10
                TargetVelocity=v_max*3.6;%km/h
            else
                TargetVelocity=-20;
            end
        elseif SlowDown==6
            if d_veh2Signstopline<=dislevel1+dist_wait
                TargetVelocity=round(TargetVelocity*3.6);%km/h
            elseif d_veh2Signstopline<=dislevel1+dislevel2+dist_wait
                TargetVelocity=30;%km/h
            elseif d_veh2Signstopline<=dislevel1+dislevel2+dislevel3+dist_wait
                TargetVelocity=40;%km/h
            else
                TargetVelocity=-20;
            end
        elseif SlowDown==2 %Ramp
            %         if wait_AvoidVehicle==1
            if d_veh2Rampstopline<=dislevel1+dist_wait
                TargetVelocity=round(TargetVelocity*3.6);%km/h
            elseif d_veh2Rampstopline<=dislevel1+dislevel2+dist_wait
                TargetVelocity=30;%km/h
            elseif d_veh2Rampstopline<=dislevel1+dislevel2+dislevel3+dist_wait
                TargetVelocity=40;%km/h
            else
                TargetVelocity=-20;
            end
            %         elseif speed>=min(v_max_SpeedPlanAvoidVehicle,v_soll_SpeedPlanAvoidVehicle)
            %             TargetSpeed=min(v_max_SpeedPlanAvoidVehicle,v_soll_SpeedPlanAvoidVehicle)*3.6;%km/h
            %         else
            %             TargetSpeed=-20;
            %         end
        elseif SlowDown==3 %oncommingcar
            if d_veh2waitingArea<=dislevel1+dist_wait
                TargetVelocity=round(TargetVelocity*3.6);%km/h
            elseif d_veh2waitingArea<=dislevel1+dislevel2+dist_wait
                TargetVelocity=30;%km/h
            elseif d_veh2waitingArea<=dislevel1+dislevel2+dislevel3+dist_wait
                TargetVelocity=40;%km/h
            else
                TargetVelocity=-20;
            end
        elseif SlowDown==1 %ped
            if  wait_ped==1
                if d_veh2stopline_ped<=dislevel1+dist_wait
                    TargetVelocity=round(TargetVelocity*3.6);%km/h
                elseif d_veh2stopline_ped<=dislevel1+dislevel2+dist_wait
                    TargetVelocity=30;%km/h
                elseif d_veh2stopline_ped<=dislevel1+dislevel2+dislevel3+dist_wait
                    TargetVelocity=40;%km/h
                else
                    TargetVelocity=-20;
                end
            else
                if d_veh2cross<=dislevel1+dislevel2+dist_wait %人行道限速
                    TargetVelocity=30;%km/h
                elseif d_veh2cross<=dislevel1+dislevel2+dislevel3+dist_wait
                    TargetVelocity=40;%km/h
                else
                    TargetVelocity=-20;
                end
            end
        elseif SlowDown==4 %trafficLight
            if wait_TrafficLight==1
                if d_veh2Intstopline<=dislevel1+dist_wait
                    TargetVelocity=round(TargetVelocity*3.6);%km/h
                elseif d_veh2Intstopline<=dislevel1+dislevel2+dist_wait && speed>=32/3.6
                    TargetVelocity=30;%km/h
                elseif d_veh2Intstopline<=dislevel1+dislevel2+dislevel3+dist_wait && speed>=42/3.6
                    TargetVelocity=40;%km/h
                else
                    TargetVelocity=-20;
                end
            elseif speed>=32/3.6
                if d_veh2Intstopline<=dislevel1+dislevel2+dist_wait && speed>=32/3.6
                    TargetVelocity=30;%km/h
                elseif d_veh2Intstopline<=dislevel1+dislevel2+dislevel3+dist_wait && speed>=42/3.6
                    TargetVelocity=40;%km/h
                else
                    TargetVelocity=-20;
                end
            else
                TargetVelocity=-20;
            end
        elseif SlowDown==8%靠边停车
            if d_veh2goal<=dislevel1+dist_wait
                TargetVelocity=round(TargetVelocity*3.6);%km/h
            elseif d_veh2goal<=dislevel1+dislevel2+dist_wait
                TargetVelocity=30;%km/h
            elseif d_veh2goal<=dislevel1+dislevel2+dislevel3+dist_wait
                TargetVelocity=40;%km/h
            else
                TargetVelocity=-20;
            end
        else
            SlowDown=int16(0);
            TargetVelocity=-20;
        end
        if TargetVelocity==-20
            SlowDown=int16(0);
        end
    elseif CurrentLaneFrontDis<dlimit&&Wait==0%wait时速度默认值
        SlowDown=int16(7);
        if CurrentLaneFrontVel<=0.1&&CurrentLaneFrontDis>=dist_wait
            if CurrentLaneFrontDis<=dislevel1+dist_wait
                TargetVelocity=round(TargetVelocity*3.6);%km/h
            elseif CurrentLaneFrontDis<=dislevel1+dislevel2+dist_wait
                TargetVelocity=30;%km/h
            elseif CurrentLaneFrontDis<=dislevel1+dislevel2+dislevel3+dist_wait
                TargetVelocity=40;%km/h
            else
                TargetVelocity=-20;
            end
        else
            TargetVelocity=CurrentLaneFrontVel*3.6;
        end
    else%wait时速度默认值
        SlowDown=int16(0);
        TargetVelocity=-20;
    end
end
%% start
if dec_start==0
    if Wait>=1&&speed<=1.5/3.6
        dec_start=int16(1);
    end
else
    if dir_start==1&&speed>=idle_speed/3.6
        dec_start=int16(0);
    end
end
if dec_start==1
    if a_soll>0&&Wait==0
        dir_start=int16(1);
    else
        dir_start=int16(0);
    end
else
    dir_start=int16(0);
end
Start=dir_start;
%AEBActive
if AEBActive>0
    Decision.LaneChange=int16(0);
    SlowDown=int16(0);
    TargetVelocity=double(-20);
    TargetSpeed=double(-20);
    Wait=int16(0);
    WaitDistance=double(200);
    Start=int16(0);
end
%精度
TargetVelocity=round(TargetVelocity);%km/h
TargetSpeed=round(100*TargetSpeed)/100;%m/s
WaitDistance=round(10*WaitDistance)/10;%m
%output
if PlannerLevel==2
   Decision.TargetSpeed =  TargetSpeed;%m/s
elseif PlannerLevel==3
   Decision.TargetSpeed = TargetVelocity/3.6;%m/s
end
Decision.states=decision_states;
Decision.Wait=Wait;
Decision.WaitDistance=WaitDistance+0.5*l_veh;%m 车中心到停止线距离
Decision.SlowDown=SlowDown;
Decision.Start=Start;
Decision.AEBactive=AEBActive;

Decision.TargetGear=TargetGear;
Decision.a_soll=a_sollTargetSpeed;
%全局变量
GlobVars.Decider.dec_start=dec_start;
GlobVars.Decider.dir_start=dir_start;
GlobVars.Decider.wait_pullover=wait_pullover;
GlobVars.Decider.distBehindGoal=distBehindGoal;
GlobVars.Decider.dec_follow=dec_follow;
GlobVars.Decider.countLaneChangeDecider=CountLaneChange;
GlobVars.Decider.currentTargetLaneIndexDecider=CurrentTargetLaneIndex;
end