function [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
    dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
    PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,TargetLaneIndexOpposite,TargetGear,TurnAroundState,TypeOfTurnAround,...,
    AEBActive,TurnAroundActive]=...,
    UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
    CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
    pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
    w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
    d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,DurationLaneChange_RePlan,...,
    LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex,TurningRadius,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLaneCurrent,s_turnaround_border,...,
    PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,...
    TurnAroundState,TargetLaneIndexOpposite,CurrentGear,TypeOfTurnAround,AEBActive,...,
    LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive)
TargetGear=CurrentGear;
% 避让故障车功能（搜寻本车所在link前方故障车）
BackupTargetLaneIndex=-1;
if isempty(LanesWithFail)==0
    [TargetLaneIndex,BackupTargetLaneIndex]=LaneSelectionWithBlockedLanes(NumOfLanes,LanesWithFail,TargetLaneIndex,CurrentLaneIndex);
end
        
a_soll=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);

% 紧急停车决策
[AEBActive,wait_ped,wait_AvoidVehicle,wait_avoidOncomingVehicle,wait_TrafficLight]=AEBDecision(AEBActive,d_veh2cross,wait_ped,wait_AvoidVehicle,wait_avoidOncomingVehicle,speed,d_veh2stopline,...,
    d_veh2waitingArea,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,d_veh2int,greenLight,wait_TrafficLight);
% PrePedestrianActive=PedestrianActive;

if AEBActive~=0
    % a_soll=min([ACC(0,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0),a_soll]);
    a_soll=min([-4*sign(speed),a_soll]);
end
if PedestrianActive
    [a_soll_SpeedPlanAvoidPedestrian,dec_ped,wait_ped]=SpeedPlanAvoidPedestrian(speed,dec_ped,d_veh2cross,w_cross,wait_ped,s_ped,v_ped,CurrentLaneFrontDis,CurrentLaneFrontVel,v_max);
    a_soll=min([a_soll_SpeedPlanAvoidPedestrian,a_soll]);
else
    if dec_ped~=0
        dec_ped=0;
    end
    if wait_ped~=0
        dec_ped=0;
    end
end
if TrafficLightActive
    [a_soll_TrafficLightActive,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight]=SpeedPlanTrafficLight(speed,dec_fol_TrafficLight,dec_bre_TrafficLight,d_veh2int,wait_TrafficLight,...,
        CurrentLaneFrontDis,CurrentLaneFrontVel,greenLight,time2nextSwitch);
    a_soll=min([a_soll_TrafficLightActive,a_soll]);
else
    if dec_fol_TrafficLight~=0
        dec_fol_TrafficLight=0;
    end
    if dec_bre_TrafficLight~=0
        dec_bre_TrafficLight=0;
    end
    if wait_TrafficLight~=0
        wait_TrafficLight=0;
    end
end
if VehicleCrossingActive
    [a_soll_SpeedPlanAvoidVehicle,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle]=SpeedPlanAvoidVehicle(speed,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,d_veh2converge,d_veh2stopline,...,
        wait_AvoidVehicle,CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle);
    a_soll=min([a_soll_SpeedPlanAvoidVehicle,a_soll]);
else
    if dec_fol_AvoidVehicle~=0
        dec_fol_AvoidVehicle=0;
    end
    if dec_bre_AvoidVehicle~=0
        dec_bre_AvoidVehicle=0;
    end
    if wait_AvoidVehicle~=0
        wait_AvoidVehicle=0;
    end
end
if VehicleOncomingActive
    [a_soll_SpeedPlanAvoidOncomingVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle]=SpeedPlanAvoidOncomingVehicle(speed,dec_avoidOncomingVehicle,d_veh2waitingArea,...,
        wait_avoidOncomingVehicle,CurrentLaneFrontDis,CurrentLaneFrontVel,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,...,
        s_veh1apostrophe3,v_max);
    a_soll=min([a_soll_SpeedPlanAvoidOncomingVehicle,a_soll]);
else
    if dec_avoidOncomingVehicle~=0
        dec_avoidOncomingVehicle=0;
    end
    if wait_avoidOncomingVehicle~=0
        wait_avoidOncomingVehicle=0;
    end
end
if LaneChangeActive
    t_re=0.5;
    l_veh=5;
    index_accel=0.25;
    a_max_comfort=1;
    a_min_comfort=-1;
    a_min=-3.5;
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
    S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh]);
    % if DurationLaneChange~=0 
%     if (CurrentLaneIndex~=TargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) )  
%         a=1;
%     end
%     a_soll_TrajPlanLaneChange=99;
    % if (CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) ) || DurationLaneChange_RePlan~=0
    if (CurrentLaneIndex~=CurrentTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) ) || DurationLaneChange_RePlan~=0
        %  if abs(pos_l-pos_l_CurrentLane)>0.3
        [traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan]=...,
            TrajPlanLaneChange_RePlan(speed,pos_s,pos_l,pos_l_CurrentLane,CurrentLaneFrontDis,CurrentLaneFrontVel,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan);
        a_soll_TrajPlanLaneChange=100;
        % end
        CountLaneChange=0*CountLaneChange;
        DurationLaneChange=0*DurationLaneChange;
    else
        [a_soll_TrajPlanLaneChange,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,CurrentTargetLaneIndex]=...,
            TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,speed,...,
            pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,BackupTargetLaneIndex,t_lc_traj,d_veh2int,w_lane_left,w_lane_right,v_max,LanesWithFail,...,
            CurrentTargetLaneIndex);
    end
    if a_soll_TrajPlanLaneChange~=100
        a_soll=min([a_soll_TrajPlanLaneChange,a_soll]);
    else
        a_soll=100;
    end
else
    if CountLaneChange~=0
        CountLaneChange=0;
    end
    if DurationLaneChange~=0
        DurationLaneChange=0;
    end
end
if TurnAroundActive
    [a_soll_TrajPlanTurnAround,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,...,
    TargetLaneIndexOpposite,TargetGear,TurnAroundState,TypeOfTurnAround,TurnAroundActive,AEBActive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,TurningRadius,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLaneCurrent,s_turnaround_border,...,
    PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,...,
    CurrentLaneIndex,v_max,a_soll,wait_TrafficLight,CurrentGear,TypeOfTurnAround,TurnAroundState,TargetLaneIndexOpposite,TurnAroundActive,AEBActive);
    if a_soll_TrajPlanTurnAround~=100
        a_soll=min([a_soll_TrajPlanTurnAround,a_soll]);
    else
        a_soll=100;
    end
else
    if dec_trunAround~=0
        dec_trunAround=0;
    end
    if TurnAroundState~=0
        TurnAroundState=0;
    end
    if TargetLaneIndexOpposite~=0
        TargetLaneIndexOpposite=0;
    end
    if wait_turnAround~=0
        wait_turnAround=0;
    end
    if TypeOfTurnAround~=0
        TypeOfTurnAround=0;
    end
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