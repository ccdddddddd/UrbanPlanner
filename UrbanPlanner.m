function [Trajectory,Decision,Refline,GlobVars]=UrbanPlanner(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,...,
    TrafficLightInfo,AvoOncomingVehInfo,AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,...,
    VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,glosaActive,PlannerLevel,GlobVars,CalibrationVars,Parameters)
%入参
CurrentLaneFrontDis = BasicsInfo.CurrentLaneFrontDis;
CurrentLaneFrontVel = BasicsInfo.CurrentLaneFrontVel;
CurrentLaneFrontLen = BasicsInfo.CurrentLaneFrontLen;
pos_s = BasicsInfo.pos_s;
pos_l = BasicsInfo.pos_l;
pos_psi=BasicsInfo.pos_psi;
if BasicsInfo.d_veh2goal<60 && BasicsInfo.GoalLaneIndex==BasicsInfo.CurrentLaneIndex
    pos_l_CurrentLane = BasicsInfo.pos_l_CurrentLane-(0.5*BasicsInfo.WidthOfLanes(BasicsInfo.GoalLaneIndex)-0.5*Parameters.w_veh-0.2);
else
    pos_l_CurrentLane = BasicsInfo.pos_l_CurrentLane;
end
if BasicsInfo.d_veh2goal<60 
    TargetLaneIndex=min(length(BasicsInfo.WidthOfLanes(BasicsInfo.WidthOfLanes>0)),BasicsInfo.GoalLaneIndex);
else
    TargetLaneIndex = BasicsInfo.TargetLaneIndex;
end
CurrentLaneIndex = BasicsInfo.CurrentLaneIndex;
% d_veh2int = BasicsInform.d_veh2int;
WidthOfLanes = BasicsInfo.WidthOfLanes;

v_max = BasicsInfo.v_max;
speed = ChassisInfo.speed;
CurrentGear = ChassisInfo.CurrentGear;
LanesWithFail = AvoFailVehInfo.LanesWithFail;
FailLaneindex = AvoFailVehInfo.FailLaneindex;% 故障车所在车道序号,数组大小5
FailLaneFrontDis = AvoFailVehInfo.FailLaneFrontDis;
FailLaneFrontVel = AvoFailVehInfo.FailLaneFrontVel;
FailLaneFrontLen = AvoFailVehInfo.FailLaneFrontLen;
LeftLaneBehindDis = LaneChangeInfo.LeftLaneBehindDis;
LeftLaneBehindVel = LaneChangeInfo.LeftLaneBehindVel;
LeftLaneFrontDis = LaneChangeInfo.LeftLaneFrontDis;
LeftLaneFrontVel = LaneChangeInfo.LeftLaneFrontVel;
RightLaneBehindDis = LaneChangeInfo.RightLaneBehindDis;
RightLaneBehindVel = LaneChangeInfo.RightLaneBehindVel;
RightLaneFrontDis = LaneChangeInfo.RightLaneFrontDis;
RightLaneFrontVel = LaneChangeInfo.RightLaneFrontVel;
LeftLaneFrontLen = LaneChangeInfo.LeftLaneFrontLen;
RightLaneFrontLen = LaneChangeInfo.RightLaneFrontLen;
d_veh2int = LaneChangeInfo.d_veh2int;
DurationLaneChange=GlobVars.TrajPlanLaneChange.DurationLaneChange;
DurationLaneChange_RePlan=GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan;
% TargetLaneBehindDisAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle;
% TargetLaneBehindVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle;
% TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle;
% TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle;

% AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的前车数据放入数组，默认值为200
% AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle“大小为4的数组”，AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(i)对应AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(i)，默认值为20
[~,NumTargetLaneFront]=min(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle);
TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(NumTargetLaneFront); 
TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(NumTargetLaneFront); 
TargetLaneFrontLenAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle(NumTargetLaneFront);
t_TargetLaneFront2int=(AvoMainRoVehInfo.d_veh2converge-AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle)./(eps+AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle);
TargetLaneFrontDisAvoidVehicleList=AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
TargetLaneFrontVelAvoidVehicleList=AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
TargetLaneFrontLenAvoidVehicleList=AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
if ~isempty(TargetLaneFrontDisAvoidVehicleList)
    [~,NumTargetLaneFront]=min(TargetLaneFrontDisAvoidVehicleList);
    TargetLaneFrontDisAvoidVehicle = TargetLaneFrontDisAvoidVehicleList(NumTargetLaneFront);
    TargetLaneFrontVelAvoidVehicle = TargetLaneFrontVelAvoidVehicleList(NumTargetLaneFront);
    TargetLaneFrontLenAvoidVehicle = TargetLaneFrontLenAvoidVehicleList(NumTargetLaneFront);
end
% AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的后车数据放入数组，默认值为-200
% AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle为“大小为4的数组”，AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(i)对应AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(i)，默认值为20
TargetLaneBehindDisAvoidVehicle=-200;
TargetLaneBehindVelAvoidVehicle=20;
TargetLaneBehindLenAvoidVehicle=5;
TargetLaneBehindDisAvoidVehicleList=AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle>-200);
TargetLaneBehindVelAvoidVehicleList=AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle>-200);
TargetLaneBehindLenAvoidVehicleList=AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle(AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle>-200);
if ~isempty(TargetLaneBehindDisAvoidVehicleList)
    [~,NumTargetLaneBehind]=min((AvoMainRoVehInfo.d_veh2converge-TargetLaneBehindDisAvoidVehicleList)./(eps+TargetLaneBehindVelAvoidVehicleList));
    TargetLaneBehindDisAvoidVehicle = TargetLaneBehindDisAvoidVehicleList(NumTargetLaneBehind);
    TargetLaneBehindVelAvoidVehicle = TargetLaneBehindVelAvoidVehicleList(NumTargetLaneBehind);
    TargetLaneBehindLenAvoidVehicle = TargetLaneBehindLenAvoidVehicleList(NumTargetLaneBehind);
end
d_veh2converge = AvoMainRoVehInfo.d_veh2converge;
% d_veh2stopline = AvoMainRoVehInform.d_veh2stopline;
d_veh2waitingArea = AvoOncomingVehInfo.d_veh2waitingArea;
s_veh1 = AvoOncomingVehInfo.s_veh;
v_veh1 = AvoOncomingVehInfo.v_veh;
d_veh2conflict = AvoOncomingVehInfo.d_veh2conflict;
s_vehapostrophe = AvoOncomingVehInfo.s_vehapostrophe;
l_vehapostrophe=AvoOncomingVehInfo.l_vehapostrophe;
d_veh2cross = AvoPedInfo.d_veh2cross;
w_cross = AvoPedInfo.w_cross;
s_ped = AvoPedInfo.s_ped;
v_ped = AvoPedInfo.v_ped;
l_ped = AvoPedInfo.l_ped;
psi_ped = AvoPedInfo.psi_ped;
greenLight = TrafficLightInfo.greenLight;
time2nextSwitch = TrafficLightInfo.time2nextSwitch;
% d_veh2stopline = TrafficLightInform.d_veh2stopline;
NumOfLanesOpposite = TurnAroundInfo.NumOfLanesOpposite;
WidthOfLanesOpposite = TurnAroundInfo.WidthOfLanesOpposite;
WidthOfGap = TurnAroundInfo.WidthOfGap;
s_turnaround_border = TurnAroundInfo.s_turnaround_border;
IndexOfLaneOppositeCar = TurnAroundInfo.IndexOfLaneOppositeCar;
SpeedOppositeCar = TurnAroundInfo.SpeedOppositeCar;
PosSOppositeCar = TurnAroundInfo.PosSOppositeCar;
IndexOfLaneCodirectCar = TurnAroundInfo.IndexOfLaneCodirectCar;
SpeedCodirectCar = TurnAroundInfo.SpeedCodirectCar;
PosSCodirectCar = TurnAroundInfo.PosSCodirectCar;
LengthOppositeCar = TurnAroundInfo.LengthOppositeCar;
LengthCodirectCar = TurnAroundInfo.LengthCodirectCar;

AEBActive = GlobVars.AEBDecision.AEBActive;
%---------------------------------------------------------
CurrentTargetLaneIndex=GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex;
t_lc_traj=GlobVars.TrajPlanLaneChange.t_lc_traj;
LaneChangePath=GlobVars.TrajPlanLaneChange.LaneChangePath;
TargetGear=CurrentGear;
%-------------------------------------------------------
traj_s=zeros([1 80],'double');
traj_l=zeros([1 80],'double');
traj_psi=zeros([1 80],'double');
traj_vs=zeros([1 80],'double');
traj_vl=zeros([1 80],'double');
traj_omega=zeros([1 80],'double');
%掉头参考线
Refline.NumRefLaneTurnAround=0;
Refline.SRefLaneTurnAround=zeros([1 100],'double');
Refline.LRefLaneTurnAround=zeros([1 100],'double');
Refline.TurnAroundReflineState=int16(0);

if TurnAroundActive==1
    GlobVars.TrajPlanTurnAround.TurnAroundActive=int16(1);
end
TurnAroundActive=GlobVars.TrajPlanTurnAround.TurnAroundActive;
% Environmental car
CurrentLaneFrontDis = CurrentLaneFrontDis-CurrentLaneFrontLen;
s_vehapostrophe=s_vehapostrophe+l_vehapostrophe;
RightLaneFrontDis=RightLaneFrontDis-RightLaneFrontLen;
LeftLaneFrontDis=LeftLaneFrontDis-LeftLaneFrontLen;

% 避让故障车功能（搜寻本车所在link前方故障车）
BackupTargetLaneIndex=int16(-1);
a_soll_Fail=100;
if any(LanesWithFail)%避让故障车
    [TargetLaneIndex,BackupTargetLaneIndex]=LaneSelectionWithBlockedLanes(WidthOfLanes,LanesWithFail,TargetLaneIndex,CurrentLaneIndex);
end
if BasicsInfo.d_veh2goal<60%靠边停车
    if CurrentLaneIndex~=TargetLaneIndex%未在目标车道较晚减速
        if BasicsInfo.d_veh2goal<((CalibrationVars.TrajPlanLaneChange.v_max_int.^2-v_max.^2)/(2*(-1.5))+CalibrationVars.TrajPlanLaneChange.v_max_int*CalibrationVars.TrajPlanLaneChange.t_permit)
            a_soll_veh2goal=ACC(30/3.6,0,BasicsInfo.d_veh2goal+CalibrationVars.ACC.d_wait,speed,1,CalibrationVars);
        else
            a_soll_veh2goal=100;
        end
    else
        a_soll_veh2goal=ACC(v_max,0,BasicsInfo.d_veh2goal+CalibrationVars.ACC.d_wait,speed,1,CalibrationVars);
    end
else
    a_soll_veh2goal=100;
end
a_soll_ACC=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0,CalibrationVars);

if any(FailLaneindex)%异常车避免碰撞
    for i=1:length(FailLaneindex)
        if  FailLaneindex(i)~=0&&FailLaneindex(i)==CurrentLaneIndex
            a_soll_fail = ACC(v_max,FailLaneFrontVel(i),FailLaneFrontDis(i)-FailLaneFrontLen(i),speed,0,CalibrationVars);
            a_soll_Fail=min(a_soll_Fail,a_soll_fail);
        end
    end
end
a_soll=min([a_soll_Fail,a_soll_ACC,a_soll_veh2goal]);
if PedestrianActive
    [a_soll_SpeedPlanAvoidPedestrian,d_veh2stopline_ped,GlobVars]=SpeedPlanAvoidPedestrian(pos_s,speed,d_veh2cross,w_cross,s_ped,l_ped,v_ped,psi_ped,CurrentLaneFrontDis,CurrentLaneFrontVel,v_max,GlobVars,Parameters,CalibrationVars);
    a_soll=min([a_soll_SpeedPlanAvoidPedestrian,a_soll]);
else
    a_soll_SpeedPlanAvoidPedestrian=100;
    d_veh2stopline_ped=double(200);
    if GlobVars.SpeedPlanAvoidPedestrian.dec_ped~=0
        GlobVars.SpeedPlanAvoidPedestrian.dec_ped=int16(0);
    end
    if GlobVars.SpeedPlanAvoidPedestrian.wait_ped~=0
        GlobVars.SpeedPlanAvoidPedestrian.dec_ped=int16(0);
    end
end

[AEBActive,GlobVars]=AEBDecision(AEBActive,speed,d_veh2stopline_ped,AvoMainRoVehInfo.d_veh2stopline,d_veh2waitingArea,s_veh1,v_veh1,d_veh2conflict,s_vehapostrophe,...,
    TrafficLightInfo.d_veh2stopline,greenLight,GlobVars,CalibrationVars,Parameters);
% PrePedestrianActive=PedestrianActive;

if AEBActive~=0
    % a_soll=min([ACC(0,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0),a_soll]);
    a_soll=min([-4*sign(speed),a_soll]);
end

if GlobVars.SpeedPlanStopSign.wait_stopsign==0%停车让行
    if StopSignInfo.d_veh2stopline<60 && StopSignInfo.d_veh2stopline>=1
        GlobVars.SpeedPlanStopSign.wait_stopsign=int16(1);
    end
else
    if (speed<=0.05 && StopSignInfo.d_veh2stopline<1) || StopSignInfo.d_veh2stopline<0 || StopSignInfo.d_veh2stopline>=200
        GlobVars.SpeedPlanStopSign.wait_stopsign=int16(0);
    end
end
if GlobVars.SpeedPlanStopSign.wait_stopsign==1
    a_soll=min([ACC(v_max,0,max([0 StopSignInfo.d_veh2stopline+CalibrationVars.ACC.d_wait]),speed,0,CalibrationVars),a_soll]);
    VehicleCrossingActive=int16(0);
end   

if TrafficLightActive
    [a_soll_TrafficLightActive,GlobVars]=SpeedPlanTrafficLight(speed,TrafficLightInfo.d_veh2stopline,CurrentLaneFrontDis,CurrentLaneFrontVel,greenLight,time2nextSwitch,v_max,GlobVars,Parameters,CalibrationVars);
    a_soll=min([a_soll_TrafficLightActive,a_soll]);
else
    a_soll_TrafficLightActive=100;
    if GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight~=0
        GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight=int16(0);
    end
    if GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight~=0
        GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight=int16(0);
    end
    if GlobVars.SpeedPlanTrafficLight.wait_TrafficLight~=0
        GlobVars.SpeedPlanTrafficLight.wait_TrafficLight=int16(0);
    end
end
    
if VehicleCrossingActive
    [a_soll_SpeedPlanAvoidVehicle,GlobVars]=SpeedPlanAvoidVehicle(speed,d_veh2converge,AvoMainRoVehInfo.d_veh2stopline, BasicsInfo.CurrentLaneFrontDis, BasicsInfo.CurrentLaneFrontVel,...
         BasicsInfo.CurrentLaneFrontLen, TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneFrontLenAvoidVehicle,TargetLaneBehindDisAvoidVehicle,...
        TargetLaneBehindVelAvoidVehicle,TargetLaneBehindLenAvoidVehicle,GlobVars,CalibrationVars,Parameters);
    a_soll=min([a_soll_SpeedPlanAvoidVehicle,a_soll]);
else
    a_soll_SpeedPlanAvoidVehicle=100;
    if GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle~=0
        GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle=int16(0);
    end
    if GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle~=0
        GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle=int16(0);
    end
    if GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle~=0
        GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle=int16(0);
    end
end
if VehicleOncomingActive
    [a_soll_SpeedPlanAvoidOncomingVehicle,GlobVars]=SpeedPlanAvoidOncomingVehicle(speed,d_veh2waitingArea,...,
        CurrentLaneFrontDis,CurrentLaneFrontVel,s_veh1,v_veh1,d_veh2conflict,s_vehapostrophe,v_max,GlobVars,CalibrationVars,Parameters);
    a_soll=min([a_soll_SpeedPlanAvoidOncomingVehicle,a_soll]);
else
    a_soll_SpeedPlanAvoidOncomingVehicle=100;
    if GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle~=0
        GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle=int16(0);
    end
    if GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle~=0
       GlobVars.SpeedPlanAvoidOncomingVehicle. wait_avoidOncomingVehicle=int16(0);
    end
end
if LaneChangeActive
    t_re=CalibrationVars.TrajPlanLaneChange.t_re;%0.5;
    l_veh=Parameters.l_veh;%5;
    index_accel=CalibrationVars.TrajPlanLaneChange.index_accel;%0.25;
    a_max_comfort=CalibrationVars.TrajPlanLaneChange.a_max_comfort;%1;
    a_min_comfort=CalibrationVars.TrajPlanLaneChange.a_min_comfort;%-1;
    a_min=CalibrationVars.TrajPlanLaneChange.a_min;%-3.5;
    if CurrentTargetLaneIndex<=CurrentLaneIndex
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
    V_end=LaneChangePath(round(t_lc_traj/0.05),4);
    S_end=LaneChangePath(round(t_lc_traj/0.05),1);
    t_remain=max([t_lc_traj-double(DurationLaneChange-1)*0.1 0]);
    s_b=TargetLaneBehindDis+pos_s;
    v_b=TargetLaneBehindVel;
    s_c=TargetLaneFrontDis+pos_s;
    v_c=TargetLaneFrontVel;
    V_c_end=max([0 v_c+(index_accel*a_min_comfort)*t_remain]);
    S_c_end=s_c+0.5*(V_c_end+v_c)*t_remain;
    V_b_end=max([0 v_b+(index_accel*a_max_comfort)*t_remain]);
    S_b_end=s_b+0.5*(V_b_end+v_b)*t_remain;
    
    S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh]);
%     S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh]);
    S_max=min([S_c_end-t_re*V_end S_c_end-(V_c_end.^2-V_end.^2)/(2*a_min)]);%20220706
    % S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)]);%20220706

    % if DurationLaneChange~=0 
%     if (CurrentLaneIndex~=TargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) )  
%         a=1;
%     end
%     a_soll_TrajPlanLaneChange=99;
    % if (CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) ) || DurationLaneChange_RePlan~=0
    % if (CurrentLaneIndex~=CurrentTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max))|| ...,
            % (DurationLaneChange_RePlan~=0 && ~(BasicsInfo.d_veh2goal<40 && CurrentLaneIndex==BasicsInfo.GoalLaneIndex))
    if (CurrentLaneIndex~=CurrentTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max))%判断换道条件是否满足，不满足换道终止
        %  if abs(pos_l-pos_l_CurrentLane)>0.3
%         [traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
%             TrajPlanLaneChange_RePlan(speed,pos_s,pos_l,pos_l_CurrentLane,WidthOfLanes,CurrentLaneIndex,CurrentLaneFrontDis,CurrentLaneFrontVel,GlobVars,CalibrationVars,Parameters);
        % a_soll_TrajPlanLaneChange=100;
        % end
        GlobVars.TrajPlanLaneChange.CountLaneChange=0*GlobVars.TrajPlanLaneChange.CountLaneChange;%换道终止
        GlobVars.TrajPlanLaneChange.DurationLaneChange=0*DurationLaneChange;
        DurationLaneChange=0;
    elseif DurationLaneChange_RePlan==0
        [a_soll_TrajPlanLaneChange,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
            TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,speed,...,
            pos_s,pos_l_CurrentLane,pos_l,CurrentLaneIndex,TargetLaneIndex,BasicsInfo.GoalLaneIndex,BackupTargetLaneIndex,d_veh2int,BasicsInfo.d_veh2goal,WidthOfLanes,v_max,LanesWithFail,...,
            GlobVars,CalibrationVars,Parameters);
        if a_soll_TrajPlanLaneChange~=100
            a_soll=min([a_soll_TrajPlanLaneChange,a_soll]);
        else
            a_soll=100;
        end
    end

else
    if GlobVars.TrajPlanLaneChange.CountLaneChange~=0
        GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0);
    end
    if GlobVars.TrajPlanLaneChange.DurationLaneChange~=0
       GlobVars.TrajPlanLaneChange.DurationLaneChange=int16(0);
    end
end
if TurnAroundActive && DurationLaneChange==0 && DurationLaneChange_RePlan==0
    [a_soll_TrajPlanTurnAround,a_sollTurnAround2Decider,Refline,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars,TargetGear,TurnAroundActive,AEBActive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLanes,s_turnaround_border,...,
    IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,LengthOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,LengthCodirectCar,CurrentLaneIndex,v_max,a_soll,CurrentGear,TurnAroundActive,AEBActive,...,
    GlobVars,CalibrationVars,Parameters);
    if a_soll_TrajPlanTurnAround~=100
        a_soll=min([a_soll_TrajPlanTurnAround,a_soll]);
    else
        a_soll=100;
    end
else
    a_sollTurnAround2Decider=100;
    if GlobVars.TrajPlanTurnAround.dec_trunAround~=0
        GlobVars.TrajPlanTurnAround.dec_trunAround=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.TurnAroundState~=0
        GlobVars.TrajPlanTurnAround.TurnAroundState=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.TargetLaneIndexOpposite~=0
        GlobVars.TrajPlanTurnAround.TargetLaneIndexOpposite=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.wait_turnAround~=0
        GlobVars.TrajPlanTurnAround.wait_turnAround=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.TypeOfTurnAround~=0
        GlobVars.TrajPlanTurnAround.TypeOfTurnAround=int16(0);
    end
end
%更新掉头参考线标志位
if GlobVars.TrajPlanTurnAround.ReflineSend~=0&&GlobVars.TrajPlanTurnAround.ReflineLend~=0&&...
   pos_s<=GlobVars.TrajPlanTurnAround.ReflineSend-Parameters.l_veh&&...
   (GlobVars.TrajPlanTurnAround.ReflineLend-Parameters.w_veh-pos_l)*(GlobVars.TrajPlanTurnAround.ReflineLend+Parameters.w_veh-pos_l)<0
    Refline.TurnAroundReflineState=int16(0);
    GlobVars.TrajPlanTurnAround.ReflineSend=0;
    GlobVars.TrajPlanTurnAround.ReflineLend=0;
elseif GlobVars.TrajPlanTurnAround.ReflineSend~=0
    Refline.TurnAroundReflineState=int16(1);
end
% 车偏离参考线轨迹规划（靠边停车的右偏轨迹规划，换道重归划）
% if PlannerLevel==1&&(BasicsInfo.d_veh2goal<40 && CurrentLaneIndex==BasicsInfo.GoalLaneIndex) && ...,
if a_soll~=100 && PlannerLevel==1 && ...,
        ((DurationLaneChange==0 && DurationLaneChange_RePlan==0 && (abs(pos_l-pos_l_CurrentLane)>0.3 || abs(pos_psi-90)>10)) || DurationLaneChange_RePlan~=0) 
%         ((DurationLaneChange==0 && DurationLaneChange_RePlan==0 && abs(pos_l-pos_l_CurrentLane)>0.3) || DurationLaneChange_RePlan~=0) 
%     [traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
%         TrajPlanLaneChange_RePlan(speed,pos_s,pos_l,pos_psi,pos_l_CurrentLane,WidthOfLanes,CurrentLaneIndex,CurrentLaneFrontDis,CurrentLaneFrontVel,GlobVars,CalibrationVars,Parameters);
[traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
    TrajPlanLaneChange_RePlan(a_soll,speed,pos_s,pos_l,pos_psi,pos_l_CurrentLane,GlobVars,CalibrationVars,Parameters);
    a_soll=100;
end
%Decider
if PlannerLevel==2||PlannerLevel==3
[Decision,GlobVars]=Decider(PlannerLevel,BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,StopSignInfo,...
    LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,glosaActive,AEBActive,TargetGear,a_soll_ACC,...
    a_soll_SpeedPlanAvoidPedestrian,a_soll_TrafficLightActive,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_sollTurnAround2Decider,...
    a_soll_Fail,TargetLaneIndex,BackupTargetLaneIndex,d_veh2stopline_ped,GlobVars,CalibrationVars,Parameters);   
else
    Decision.AEBactive=AEBActive;
    Decision.TargetGear=TargetGear;
    Decision.LaneChange=int16(0);
    Decision.SlowDown=int16(0);
    Decision.TargetSpeed=double(0);
    Decision.Wait=int16(0);
    Decision.WaitDistance=double(200);
    Decision.Start=int16(0);
    Decision.a_soll=a_soll;
    Decision.LaneChange=int16(0);
    Decision.PedestrianState=int16(0);
    Decision.TrafficLightState=int16(0);
    Decision.VehicleCrossingState=int16(0);
    Decision.VehicleOncomingState=int16(0);
    Decision.StopSignState=int16(0);
    Decision.FollowState=int16(0);
    Decision.PullOverState=int16(0);
    Decision.TurnAroundState=int16(0);
end
if a_soll~=100
    traj_s=zeros([1 80]);
    traj_l=zeros([1 80]);
    traj_psi=zeros([1 80]);
    traj_vs=zeros([1 80]);
    traj_vl=zeros([1 80]);
    traj_omega=zeros([1 80]);
    for count_1=1:1:80
        t_count_1=0.05*count_1;
        traj_vs(count_1)=max([0 speed+a_soll*t_count_1]);
        traj_vl(count_1)=0;
        traj_omega(count_1)=0;
        traj_l(count_1)=pos_l_CurrentLane;
        traj_psi(count_1)=90;
        if traj_vs(count_1)==0
            traj_s(count_1)=pos_s+(0-speed.^2)/(2*a_soll+eps);
        else
            traj_s(count_1)=pos_s+(traj_vs(count_1)+speed)*t_count_1/2;
        end
    end
end

Trajectory.traj_s=traj_s;
Trajectory.traj_l=traj_l;
Trajectory.traj_psi=traj_psi;
Trajectory.traj_vs=traj_vs;
Trajectory.traj_vl=traj_vl;
Trajectory.traj_omega=traj_omega;

GlobVars.AEBDecision.AEBActive=AEBActive;
GlobVars.TrajPlanTurnAround.TurnAroundActive=TurnAroundActive;
% 全局变量类型设置
GlobVars.AEBDecision.AEBActive=int16(GlobVars.AEBDecision.AEBActive);
GlobVars.TrajPlanTurnAround.dec_trunAround=int16(GlobVars.TrajPlanTurnAround.dec_trunAround);
GlobVars.TrajPlanTurnAround.wait_turnAround=int16(GlobVars.TrajPlanTurnAround.wait_turnAround);
GlobVars.TrajPlanTurnAround.TypeOfTurnAround=int16(GlobVars.TrajPlanTurnAround.TypeOfTurnAround);
GlobVars.TrajPlanTurnAround.TurnAroundState=int16(GlobVars.TrajPlanTurnAround.TurnAroundState);
GlobVars.TrajPlanTurnAround.TargetLaneIndexOpposite=int16(GlobVars.TrajPlanTurnAround.TargetLaneIndexOpposite);
GlobVars.TrajPlanTurnAround.TurnAroundActive=int16(GlobVars.TrajPlanTurnAround.TurnAroundActive);
GlobVars.SpeedPlanAvoidPedestrian.dec_ped=int16(GlobVars.SpeedPlanAvoidPedestrian.dec_ped);
GlobVars.SpeedPlanAvoidPedestrian.wait_ped=int16(GlobVars.SpeedPlanAvoidPedestrian.wait_ped);
GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight=int16(GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight);
GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight=int16(GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight);
GlobVars.SpeedPlanTrafficLight.wait_TrafficLight=int16(GlobVars.SpeedPlanTrafficLight.wait_TrafficLight);
GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle=int16(GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle);
GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle=int16(GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle);
GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle=int16(GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle);
GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle=int16(GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle);
GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle=int16(GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle);
GlobVars.TrajPlanLaneChange.CountLaneChange=int16(GlobVars.TrajPlanLaneChange.CountLaneChange);
GlobVars.TrajPlanLaneChange.DurationLaneChange=int16(GlobVars.TrajPlanLaneChange.DurationLaneChange);
GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex=int16(GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex);
GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan=int16(GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan);
% 可配置参数：t_re l_veh w_veh tau_v tau_d a_max a_min a_min_comfort
% if TrafficLightActive
% a_soll_TrafficLightActive
% end
% if VehicleCrossingActive
% a_soll_SpeedPlanAvoidVehicle
% end
% a_soll
% a_soll
% speed
end
% 函数调用
% [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol,dec_bre,wait]=...,
%     UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,TargetLaneBehindDis,TargetLaneBehindVel,TargetLaneFrontDis,TargetLaneFrontVel,speed,pos_s,pos_l,CurrentLaneIndex,...,
%     CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,w_lane,dec_ped,d_veh2cross,wait_ped,s_ped,dec_fol,dec_bre,wait,greenLight,time2nextSwitch,v_max,...,
%     LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive);
% if strcmp(current_road_ID,'12') && pos_s>210
%     if a_soll==100
%         traci.vehicle.moveToXY('S0','12', 2, traj_l(2), traj_s(2),90-traj_psi(2),2);
%     else
%         traci.vehicle.setSpeed('S0',traj_vs(2));
%         traci.vehicle.changeLane('S0',1-CurrentLaneIndex,2);
%     end
% else
%     traci.vehicle.setSpeed('S0',traj_vs(2));
% end