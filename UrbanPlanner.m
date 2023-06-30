function [Trajectory,Decision,Refline,GlobVars]=UrbanPlanner(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,...,
    TrafficLightInfo,AvoOncomingVehInfo,AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,...,
    VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,GlosaActive,PlannerLevel,GlobVars,CalibrationVars,Parameters)
%打印入参
printer=logOut;
printer.logInput(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,...,
    AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,...,
    VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,GlosaActive,PlannerLevel,GlobVars,CalibrationVars,Parameters)
%入参
CurrentLaneFrontDis = BasicsInfo.currentLaneFrontDis;
CurrentLaneFrontVel = BasicsInfo.currentLaneFrontVel;
CurrentLaneFrontLen = BasicsInfo.currentLaneFrontLen;
pos_s = BasicsInfo.pos_s;
pos_l = BasicsInfo.pos_l;
pos_psi=BasicsInfo.pos_psi;
NumOfCurrentLane=nnz(BasicsInfo.widthOfLanes);
if BasicsInfo.d_veh2goal<60 && BasicsInfo.goalLaneIndex == BasicsInfo.currentLaneIndex && NumOfCurrentLane == BasicsInfo.goalLaneIndex
    pos_l_CurrentLane = BasicsInfo.pos_l_CurrentLane-(0.5*BasicsInfo.widthOfLanes(BasicsInfo.goalLaneIndex)-0.5*Parameters.w_veh-0.2);
else
    pos_l_CurrentLane = BasicsInfo.pos_l_CurrentLane;
end
if BasicsInfo.d_veh2goal<60 
    TargetLaneIndex=min(length(BasicsInfo.widthOfLanes(BasicsInfo.widthOfLanes>0)),BasicsInfo.goalLaneIndex);
else
    TargetLaneIndex = BasicsInfo.targetLaneIndex;
end
CurrentLaneIndex = BasicsInfo.currentLaneIndex;
% d_veh2int = BasicsInform.d_veh2int;
WidthOfLanes = BasicsInfo.widthOfLanes;
v_max = BasicsInfo.v_max;
speed = ChassisInfo.speed;
CurrentGear = ChassisInfo.currentGear;
LanesWithFail = AvoFailVehInfo.lanesWithFail;
FailLaneindex = AvoFailVehInfo.failLaneindex;% 故障车所在车道序号,数组大小5
FailLaneFrontDis = AvoFailVehInfo.failLaneFrontDis;
FailLaneFrontVel = AvoFailVehInfo.failLaneFrontVel;
FailLaneFrontLen = AvoFailVehInfo.failLaneFrontLen;
LeftLaneBehindDis = LaneChangeInfo.leftLaneBehindDis;
LeftLaneBehindVel = LaneChangeInfo.leftLaneBehindVel;
LeftLaneFrontDis = LaneChangeInfo.leftLaneFrontDis;
LeftLaneFrontVel = LaneChangeInfo.leftLaneFrontVel;
RightLaneBehindDis = LaneChangeInfo.rightLaneBehindDis;
RightLaneBehindVel = LaneChangeInfo.rightLaneBehindVel;
RightLaneFrontDis = LaneChangeInfo.rightLaneFrontDis;
RightLaneFrontVel = LaneChangeInfo.rightLaneFrontVel;
LeftLaneFrontLen = LaneChangeInfo.leftLaneFrontLen;
RightLaneFrontLen = LaneChangeInfo.rightLaneFrontLen;
LeftLaneBehindLen = LaneChangeInfo.leftLaneBehindLen;
RightLaneBehindLen = LaneChangeInfo.rightLaneBehindLen;
d_veh2int = LaneChangeInfo.d_veh2int;
DurationLaneChange_RePlan=GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan;
% TargetLaneBehindDisAvoidVehicle = AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle;
% TargetLaneBehindVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle;
% TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle;
% TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle;

% AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的前车数据放入数组，默认值为200
% AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle“大小为4的数组”，AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(i)对应AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(i)，默认值为20
[~,NumTargetLaneFront]=min(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle);
TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(NumTargetLaneFront); 
TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(NumTargetLaneFront); 
TargetLaneFrontLenAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(NumTargetLaneFront);
t_TargetLaneFront2int=(AvoMainRoVehInfo.d_veh2converge-AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle)./(eps+AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle);
TargetLaneFrontDisAvoidVehicleList=AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
TargetLaneFrontVelAvoidVehicleList=AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
TargetLaneFrontLenAvoidVehicleList=AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle<200 & ...,
    t_TargetLaneFront2int<(2*AvoMainRoVehInfo.d_veh2converge)/(eps+speed));
if ~isempty(TargetLaneFrontDisAvoidVehicleList)
    [~,NumTargetLaneFront]=min(TargetLaneFrontDisAvoidVehicleList);
    TargetLaneFrontDisAvoidVehicle = TargetLaneFrontDisAvoidVehicleList(NumTargetLaneFront);
    TargetLaneFrontVelAvoidVehicle = TargetLaneFrontVelAvoidVehicleList(NumTargetLaneFront);
    TargetLaneFrontLenAvoidVehicle = TargetLaneFrontLenAvoidVehicleList(NumTargetLaneFront);
end
% AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的后车数据放入数组，默认值为-200
% AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle为“大小为4的数组”，AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(i)对应AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(i)，默认值为20
TargetLaneBehindDisAvoidVehicle=-200;
TargetLaneBehindVelAvoidVehicle=20;
TargetLaneBehindLenAvoidVehicle=5;
TargetLaneBehindDisAvoidVehicleList=AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle>-200);
TargetLaneBehindVelAvoidVehicleList=AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle>-200);
TargetLaneBehindLenAvoidVehicleList=AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle>-200);
if ~isempty(TargetLaneBehindDisAvoidVehicleList)
    [~,NumTargetLaneBehind]=min((AvoMainRoVehInfo.d_veh2converge-TargetLaneBehindDisAvoidVehicleList)./(eps+TargetLaneBehindVelAvoidVehicleList));
    TargetLaneBehindDisAvoidVehicle = TargetLaneBehindDisAvoidVehicleList(NumTargetLaneBehind);
    TargetLaneBehindVelAvoidVehicle = TargetLaneBehindVelAvoidVehicleList(NumTargetLaneBehind);
    TargetLaneBehindLenAvoidVehicle = TargetLaneBehindLenAvoidVehicleList(NumTargetLaneBehind);
end
d_veh2converge = AvoMainRoVehInfo.d_veh2converge;
d_veh2crossStopline = AvoMainRoVehInfo.d_veh2stopline;
d_veh2waitingArea = AvoOncomingVehInfo.d_veh2waitingArea;
s_veh1 = AvoOncomingVehInfo.s_veh;
v_veh1 = AvoOncomingVehInfo.v_veh;
s_veh_length=AvoOncomingVehInfo.l_veh;
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
% time2nextSwitch = TrafficLightInfo.time2nextSwitch;
time2nextSwitch = TrafficLightInfo.phase(1);
d_veh2trafficStopline = TrafficLightInfo.d_veh2stopline;
NumOfLanesOpposite = TurnAroundInfo.numOfLanesOpposite;
WidthOfLanesOpposite = TurnAroundInfo.widthOfLanesOpposite;
WidthOfGap = TurnAroundInfo.widthOfGap;
s_turnaround_border = TurnAroundInfo.s_turnaround_border;
IndexOfLaneOppositeCar = TurnAroundInfo.indexOfLaneOppositeCar;
SpeedOppositeCar = TurnAroundInfo.speedOppositeCar;
PosSOppositeCar = TurnAroundInfo.posSOppositeCar;
IndexOfLaneCodirectCar = TurnAroundInfo.indexOfLaneCodirectCar;
SpeedCodirectCar = TurnAroundInfo.speedCodirectCar;
PosSCodirectCar = TurnAroundInfo.posSCodirectCar;
LengthOppositeCar = TurnAroundInfo.lengthOppositeCar;
LengthCodirectCar = TurnAroundInfo.lengthCodirectCar;

AEBActive = GlobVars.AEBDecision.AEBActive;
%---------------------------------------------------------
TargetGear=CurrentGear;
%-------------------------------------------------------
traj_s=zeros([1 80],'double');
traj_l=zeros([1 80],'double');
traj_psi=zeros([1 80],'double');
traj_vs=zeros([1 80],'double');
traj_vl=zeros([1 80],'double');
traj_omega=zeros([1 80],'double');
planning_states=int16(0);%0:Undefined 1:终点 2:AEB触发
%掉头参考线
Refline.NumRefLaneTurnAround=int16(0);
Refline.SRefLaneTurnAround=zeros([1 100],'double');
Refline.LRefLaneTurnAround=zeros([1 100],'double');
Refline.TurnAroundReflineState=int16(0);

if TurnAroundActive==1
    GlobVars.TrajPlanTurnAround.turnAroundActive=int16(1);
end
TurnAroundActive=GlobVars.TrajPlanTurnAround.turnAroundActive;
% Environmental car
% CurrentLaneFrontDis = CurrentLaneFrontDis-CurrentLaneFrontLen;
% s_vehapostrophe=s_vehapostrophe+l_vehapostrophe;
% RightLaneFrontDis=RightLaneFrontDis-RightLaneFrontLen;
% LeftLaneFrontDis=LeftLaneFrontDis-LeftLaneFrontLen;

BasicsInfo.d_veh2goal=BasicsInfo.d_veh2goal-0.5*Parameters.l_veh;%车中心转车头
CurrentLaneFrontDis=CurrentLaneFrontDis-0.5*(CurrentLaneFrontLen+Parameters.l_veh);%车头到前车车尾距离
%避让对向车
s_veh1=s_veh1-0.5.*s_veh_length;%车中心距离转为车头距离
s_vehapostrophe=s_vehapostrophe+0.5.*l_vehapostrophe;%车中心距离转为车尾距离
d_veh2waitingArea=d_veh2waitingArea-0.5*Parameters.l_veh;%车中心距离转为车头距离
%换道入参：
d_veh2int=d_veh2int-0.5*Parameters.l_veh;%车中心距离转为车头距离
RightLaneFrontDis=RightLaneFrontDis-0.5*(RightLaneFrontLen+Parameters.l_veh);%车头到车尾距离
LeftLaneFrontDis=LeftLaneFrontDis-0.5*(LeftLaneFrontLen+Parameters.l_veh);%车头到车尾距离
RightLaneBehindDis = RightLaneBehindDis+0.5*(Parameters.l_veh-RightLaneBehindLen);%车头到车头距离
LeftLaneBehindDis = LeftLaneBehindDis+0.5*(Parameters.l_veh-LeftLaneBehindLen);%车头到车头距离
%避让同向车入参：车中心距离转为车头距离
d_veh2converge = d_veh2converge-0.5*Parameters.l_veh;
d_veh2crossStopline=d_veh2crossStopline-0.5*Parameters.l_veh;
TargetLaneFrontDisAvoidVehicle = TargetLaneFrontDisAvoidVehicle+0.5*(TargetLaneFrontLenAvoidVehicle-Parameters.l_veh);%车头到车头距离
TargetLaneBehindDisAvoidVehicle = TargetLaneBehindDisAvoidVehicle+0.5*(Parameters.l_veh-TargetLaneBehindLenAvoidVehicle);%车头到车头距离
%故障车位置：
FailLaneFrontDis=FailLaneFrontDis+0.5.*(FailLaneFrontLen-Parameters.l_veh);%车中心距离转为车头与车头距离
%停车让行停止线距离
StopSignInfo.d_veh2stopline=StopSignInfo.d_veh2stopline-0.5*Parameters.l_veh;%%车中心距离转为车头距离
%信号灯通行
d_veh2trafficStopline=d_veh2trafficStopline-0.5*Parameters.l_veh;%%车中心距离转为车头距离
%行人
d_veh2cross=d_veh2cross-0.5*Parameters.l_veh;%%车中心距离转为车头距离
s_ped=s_ped-0.5*Parameters.l_veh;%车中心距离转为车头距离
%掉头
s_turnaround_border = s_turnaround_border-0.5*Parameters.l_veh;%%车中心距离转为车头距离
PosSOppositeCar = PosSOppositeCar-0.5.*LengthOppositeCar;%对向车中心坐标转车头坐标
PosSCodirectCar = PosSCodirectCar+0.5.*LengthCodirectCar;%同向车中心坐标转车头坐标


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
        if a_soll_veh2goal<=0&&speed<0.2
            planning_states=int16(1);
        end
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
[AEBActive,GlobVars]=AEBDecision(AEBActive,speed,d_veh2stopline_ped,d_veh2crossStopline,d_veh2waitingArea,s_veh1,v_veh1,d_veh2conflict,s_vehapostrophe,...,
    d_veh2trafficStopline,greenLight,CurrentLaneFrontDis,CurrentLaneFrontVel,CurrentLaneIndex,GlobVars,CalibrationVars,Parameters);
if AEBActive~=0
    a_soll=min([-4*sign(speed),a_soll]);
    planning_states=int16(2);
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
    [a_soll_TrafficLightActive,GlobVars]=SpeedPlanTrafficLight(speed,d_veh2trafficStopline,CurrentLaneFrontDis,CurrentLaneFrontVel,greenLight,time2nextSwitch,v_max,GlobVars,Parameters,CalibrationVars);
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
    [a_soll_SpeedPlanAvoidVehicle,GlobVars]=SpeedPlanAvoidVehicle(speed,d_veh2converge,d_veh2crossStopline, CurrentLaneFrontDis+CurrentLaneFrontLen, CurrentLaneFrontVel,...
         CurrentLaneFrontLen, TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneFrontLenAvoidVehicle,TargetLaneBehindDisAvoidVehicle,...
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
a_soll_TrajPlanLaneChange=0;
if LaneChangeActive
     if DurationLaneChange_RePlan==0
        [a_soll_TrajPlanLaneChange,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
            TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,speed,...,
            pos_s,pos_l_CurrentLane,pos_l,CurrentLaneIndex,TargetLaneIndex,BasicsInfo.goalLaneIndex,BackupTargetLaneIndex,d_veh2int,BasicsInfo.d_veh2goal,WidthOfLanes,v_max,LanesWithFail,AEBActive,...,
            GlobVars,CalibrationVars,Parameters);
        a_soll=min([a_soll_TrajPlanLaneChange,a_soll]);
    end
else
    if GlobVars.TrajPlanLaneChange.countLaneChange~=0
        GlobVars.TrajPlanLaneChange.countLaneChange=int16(0);
    end
    if GlobVars.TrajPlanLaneChange.durationLaneChange~=0
       GlobVars.TrajPlanLaneChange.durationLaneChange=int16(0);
    end
end
%最近停车位置
stopdistance_array=zeros(1,8)+200;
if GlobVars.SpeedPlanAvoidPedestrian.wait_ped==1
    stopdistance_array(1)=d_veh2stopline_ped-CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped;
end
if GlobVars.SpeedPlanTrafficLight.wait_TrafficLight==1
    stopdistance_array(4)=d_veh2trafficStopline-CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline;
end
if GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle==1
    stopdistance_array(2)=d_veh2crossStopline-CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline;%应用路口停止线标定量
end
if GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle==1
    stopdistance_array(3)=d_veh2waitingArea;
end
% if TurnAroundActive==1&&GlobVars.TrajPlanTurnAround.wait_turnAround==1%激活第二帧有值
%     stopdistance_array(5)=GlobVars.TrajPlanTurnAround.PosCircle(1)-BasicsInfo.pos_s;
% end
if GlobVars.SpeedPlanStopSign.wait_stopsign==1
    stopdistance_array(6)=StopSignInfo.d_veh2stopline;
end
if CurrentLaneFrontVel<0.2
    wait=0;
    if isempty(LanesWithFail)==0
        for i=LanesWithFail
            if CurrentLaneIndex==i
                wait=-1;
            end
        end
    end
    if wait==-1
        stopdistance_array(7)=CurrentLaneFrontDis-17;
    else
        stopdistance_array(7)=CurrentLaneFrontDis-CalibrationVars.ACC.d_wait;
    end
end
stopdistance_array(8)=BasicsInfo.d_veh2goal;
stopdistance=min(stopdistance_array);
% 车偏离参考线轨迹规划（靠边停车的右偏轨迹规划，换道重归划）
if PlannerLevel==1 &&GlobVars.TrajPlanLaneChange.durationLaneChange==0 &&TurnAroundActive==0 && a_soll_TrajPlanLaneChange~=100 ...,
        && (abs(pos_l-pos_l_CurrentLane)>0.3 || abs(pos_psi-90)>10) || DurationLaneChange_RePlan~=0
[traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
    TrajPlanLaneChange_RePlan(a_soll,speed,pos_s,pos_l,pos_psi,pos_l_CurrentLane,stopdistance,BasicsInfo.sampleTime,a_soll_ACC,CurrentLaneFrontVel,GlobVars,CalibrationVars,Parameters);
end
if TurnAroundActive && GlobVars.TrajPlanLaneChange.durationLaneChange==0 && GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0
    [a_soll_TrajPlanTurnAround,a_sollTurnAround2Decider,Refline,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars,TargetGear,TurnAroundActive,AEBActive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLanes,s_turnaround_border,...,
    IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,LengthOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,LengthCodirectCar,CurrentLaneIndex,v_max,a_soll,...,
    CurrentGear,TurnAroundActive,AEBActive,stopdistance,a_soll_ACC,BasicsInfo.sampleTime,GlobVars,CalibrationVars,Parameters);
    if a_soll_TrajPlanTurnAround~=100
        a_soll=min([a_soll_TrajPlanTurnAround,a_soll]);
    end
else
    a_sollTurnAround2Decider=100;
    if GlobVars.TrajPlanTurnAround.dec_trunAround~=0
        GlobVars.TrajPlanTurnAround.dec_trunAround=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.turnAroundState~=0
        GlobVars.TrajPlanTurnAround.turnAroundState=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite~=0
        GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.wait_turnAround~=0
        GlobVars.TrajPlanTurnAround.wait_turnAround=int16(0);
    end
    if GlobVars.TrajPlanTurnAround.typeOfTurnAround~=0
        GlobVars.TrajPlanTurnAround.typeOfTurnAround=int16(0);
    end
end
%更新掉头参考线标志位
if PlannerLevel==1
    Refline.TurnAroundReflineState=TurnAroundActive;
else
    if GlobVars.TrajPlanTurnAround.reflineSend~=0&&GlobVars.TrajPlanTurnAround.reflineLend~=0&&...
            pos_s<=GlobVars.TrajPlanTurnAround.reflineSend-Parameters.l_veh&&...
            (GlobVars.TrajPlanTurnAround.reflineLend-Parameters.w_veh-pos_l)*(GlobVars.TrajPlanTurnAround.reflineLend+Parameters.w_veh-pos_l)<0
        Refline.TurnAroundReflineState=int16(0);
        GlobVars.TrajPlanTurnAround.reflineSend=0;
        GlobVars.TrajPlanTurnAround.reflineLend=0;
    elseif GlobVars.TrajPlanTurnAround.reflineSend~=0
        Refline.TurnAroundReflineState=int16(1);
    end
end

%Decider
if PlannerLevel==2||PlannerLevel==3
[Decision,GlobVars]=Decider(PlannerLevel,BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,StopSignInfo,...
    AvoFailVehInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,GlosaActive,AEBActive,TargetGear,a_soll_ACC,...
    a_soll_SpeedPlanAvoidPedestrian,a_soll_TrafficLightActive,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_sollTurnAround2Decider,...
    a_soll_Fail,TargetLaneIndex,BackupTargetLaneIndex,d_veh2stopline_ped,GlobVars,CalibrationVars,Parameters);   
else
    Decision.AEBactive=AEBActive;
    Decision.TargetGear=TargetGear;
    Decision.states=int16(0);
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
%--------------------------------------------------------------------------------------------------------------------
% 轨迹生成
if GlobVars.TrajPlanLaneChange.durationLaneChange==0 &&GlobVars.TrajPlanTurnAround.turnAroundActive==0 &&GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0 ...,
        && a_soll_TrajPlanLaneChange~=100
    IsStopSpeedPlan=0;
    %停车速度规划  停止线前停车或跟车停车场景且以最小减速度-4制动距离小于停车距离
    if stopdistance<200-0.5*Parameters.l_veh && speed.^2/8<=stopdistance && (-((4/9)*speed.^2/(2/3*stopdistance))<=a_soll_ACC || CurrentLaneFrontVel<0.2) && AEBActive==0
%         已知初速度v_0、目标停车距离s、初始加速度a_0，确定停车轨迹（匀加加速度）
%         1建立关于加加速度J和停车时间t的方程：
%         s=v_0 t+1/2 a_0 t^2+1/6 Jt^3     
%         0=v=v_0+a_0 t+1/2 Jt^2
%         2.当a_0为正，求解方程，因为t应为正数，所有只有一个解：
%         t=(-2/3 v_0+√(4/9 〖v_0〗^2+2/3 a_0 s))/(1/3 a)
%         J=(-2(v_0+a_0 t))/t^2
%         3.当a_0为负，求解方程：
%         有解的条件如下，也就是目标停车距离不大于最大停车距离（匀加加速度行驶到v=0、a=0的时候的s）：
%         s≤-(4/9 〖v_0〗^2)/(2/3 a_0 )
%         有解的情况下，存在一个（目标停车距离等于最大停车距离）或者两个解。两个解的情况下，较大的解为“先行驶经过目标停车位置，接着倒车到达目标停车位置”，不符合要求，取较小的解，如下：
%         t=(-2/3 v_0+√(4/9 〖v_0〗^2+2/3 a_0 s))/(1/3 a)
%         J=(-2(v_0+a_0 t))/t^2
        a_soll=max(a_soll,-((4/9)*speed.^2/(2/3*stopdistance)));
        [a_soll]=JerkLimit(GlobVars.Decider.a_sollpre2traj,BasicsInfo.sampleTime,a_soll,CalibrationVars);
        tend=0;
        jerk=0;
		if abs(a_soll)<=0.00001 %a_soll为0的情况
			if speed>0 && stopdistance>0
				tend=3*stopdistance/2/speed;
				jerk=-8*speed.^3/9/(stopdistance.^2);
				IsStopSpeedPlan=1;
			else
				IsStopSpeedPlan=0;
			end
        elseif a_soll>=-((4/9)*speed.^2/(2/3*stopdistance))
            tend=(3*sqrt(max(0,(4/9)*speed.^2+(2/3)*a_soll*stopdistance))-2*speed)/(a_soll+eps);
            jerk=-2*(speed+a_soll*tend)/(tend.^2);
			IsStopSpeedPlan=1;
		end
		if IsStopSpeedPlan
            for count_1=1:1:80
                t_count_1=0.05*count_1;
                if t_count_1<=tend
                    traj_vs(count_1)= speed+a_soll*t_count_1+0.5*jerk*t_count_1.^2;
                    traj_s(count_1)=pos_s+speed*t_count_1+0.5*a_soll*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
                else
                    traj_vs(count_1)=0;
                    traj_s(count_1)=pos_s+stopdistance;
                end
                traj_vl(count_1)=0;
                traj_omega(count_1)=0;
                traj_l(count_1)=pos_l_CurrentLane;
                traj_psi(count_1)=90;
            end
		end
    end
    if IsStopSpeedPlan==0
        [a_soll]=JerkLimit(GlobVars.Decider.a_sollpre2traj,BasicsInfo.sampleTime,a_soll,CalibrationVars);
        if a_soll<0
            Vend=0;
        elseif a_soll>0
            Vend=max(v_max,speed);
        else
            Vend=speed;
        end
        if abs(a_soll)<=0.001
            Vend=speed;
            tend=0;
        else
            tend=(Vend-speed)/(a_soll+eps);
        end
        for count_1=1:1:80
            t_count_1=0.05*count_1;
            traj_vl(count_1)=0;
            traj_omega(count_1)=0;
            traj_l(count_1)=pos_l_CurrentLane;
            traj_psi(count_1)=90;
            if t_count_1<=tend
                traj_vs(count_1)= speed+a_soll*t_count_1;
                traj_s(count_1)=pos_s+speed*t_count_1+0.5*a_soll*t_count_1.^2;
            else
                traj_vs(count_1)=Vend;
                traj_s(count_1)=pos_s+(Vend.^2-speed.^2)/(2*a_soll+eps)+(t_count_1-tend)*Vend;
            end
        end
    end
end
%--------------------------------------------------------------------------------------------------------------------
% if a_soll~=100
%     traj_s=zeros([1 80]);
%     traj_l=zeros([1 80]);
%     traj_psi=zeros([1 80]);
%     traj_vs=zeros([1 80]);
%     traj_vl=zeros([1 80]);
%     traj_omega=zeros([1 80]);
%     for count_1=1:1:80
%         t_count_1=0.05*count_1;
%         traj_vs(count_1)=max([0 speed+a_soll*t_count_1]);
%         traj_vl(count_1)=0;
%         traj_omega(count_1)=0;
%         traj_l(count_1)=pos_l_CurrentLane;
%         traj_psi(count_1)=90;
%         if traj_vs(count_1)==0
%             traj_s(count_1)=pos_s+(0-speed.^2)/(2*a_soll+eps);
%         else
%             traj_s(count_1)=pos_s+(traj_vs(count_1)+speed)*t_count_1/2;
%         end
%     end
% end
Trajectory.traj_s=traj_s;
Trajectory.traj_l=traj_l;
Trajectory.traj_psi=traj_psi;
Trajectory.traj_vs=traj_vs;
Trajectory.traj_vl=traj_vl;
Trajectory.traj_omega=traj_omega;
Trajectory.planning_states=planning_states;
GlobVars.AEBDecision.AEBActive=AEBActive;
GlobVars.TrajPlanTurnAround.turnAroundActive=TurnAroundActive;
% 全局变量类型设置
GlobVars.AEBDecision.AEBActive=int16(GlobVars.AEBDecision.AEBActive);
GlobVars.TrajPlanTurnAround.dec_trunAround=int16(GlobVars.TrajPlanTurnAround.dec_trunAround);
GlobVars.TrajPlanTurnAround.wait_turnAround=int16(GlobVars.TrajPlanTurnAround.wait_turnAround);
GlobVars.TrajPlanTurnAround.typeOfTurnAround=int16(GlobVars.TrajPlanTurnAround.typeOfTurnAround);
GlobVars.TrajPlanTurnAround.turnAroundState=int16(GlobVars.TrajPlanTurnAround.turnAroundState);
GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite=int16(GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite);
GlobVars.TrajPlanTurnAround.turnAroundActive=int16(GlobVars.TrajPlanTurnAround.turnAroundActive);
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
GlobVars.TrajPlanLaneChange.countLaneChange=int16(GlobVars.TrajPlanLaneChange.countLaneChange);
GlobVars.TrajPlanLaneChange.durationLaneChange=int16(GlobVars.TrajPlanLaneChange.durationLaneChange);
GlobVars.TrajPlanLaneChange.currentTargetLaneIndex=int16(GlobVars.TrajPlanLaneChange.currentTargetLaneIndex);
GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan=int16(GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan);
if GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0&&TurnAroundActive==0
    GlobVars.Decider.a_sollpre2traj=a_soll;
end
%打印出参
printer.logOutput(Trajectory,Decision,Refline,CalibrationVars)
if sum(CalibrationVars.UrbanPlanner.logTrigger)>0
    fprintf('***----------------------------------------------- log end -------------------------------------------------------***\n');
end
end