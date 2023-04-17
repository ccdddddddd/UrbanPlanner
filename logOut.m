function self=logOut
self.logInput=@logInput;
self.logOutput=@logOutput;
end
function logInput(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,...,
    AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,GlosaActive,PlannerLevel,GlobVars,CalibrationVars,Parameters)
%打印入参
if CalibrationVars.UrbanPlanner.logTrigger(1)==1
    fprintf('AvoFailVehInfo.lanesWithFail= \t');
    for i=1:length(AvoFailVehInfo.lanesWithFail)
        if i==length(AvoFailVehInfo.lanesWithFail)
            fprintf('%d\n',AvoFailVehInfo.lanesWithFail(i));
        else
            fprintf('%d\t',AvoFailVehInfo.lanesWithFail(i));
        end
    end
    fprintf('AvoFailVehInfo.failLaneindex= \t');
    for i=1:length(AvoFailVehInfo.failLaneindex)
        if i==length(AvoFailVehInfo.failLaneindex)
            fprintf('%d\n',AvoFailVehInfo.failLaneindex(i));
        else
            fprintf('%d\t',AvoFailVehInfo.failLaneindex(i));
        end
    end
    fprintf('AvoFailVehInfo.failLaneFrontDis = \t');
    for i=1:length(AvoFailVehInfo.failLaneFrontDis)
        if i==length(AvoFailVehInfo.failLaneFrontDis)
            fprintf('%f\n',AvoFailVehInfo.failLaneFrontDis(i));
        else
            fprintf('%f\t',AvoFailVehInfo.failLaneFrontDis(i));
        end
    end
    fprintf('AvoFailVehInfo.failLaneFrontVel = \t');
    for i=1:length(AvoFailVehInfo.failLaneFrontVel)
        if i==length(AvoFailVehInfo.failLaneFrontVel)
            fprintf('%f\n',AvoFailVehInfo.failLaneFrontVel(i));
        else
            fprintf('%f\t',AvoFailVehInfo.failLaneFrontVel(i));
        end
    end
    fprintf('AvoFailVehInfo.failLaneFrontLen = \t');
    for i=1:length(AvoFailVehInfo.failLaneFrontLen)
        if i==length(AvoFailVehInfo.failLaneFrontLen)
            fprintf('%f\n',AvoFailVehInfo.failLaneFrontLen(i));
        else
            fprintf('%f\t',AvoFailVehInfo.failLaneFrontLen(i));
        end
    end
end
if CalibrationVars.UrbanPlanner.logTrigger(2)==1
    fprintf('AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle = \t');
    for i=1:length(AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle)
        if i==length(AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle)
            fprintf('%f\n',AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(i));
        else
            fprintf('%f\t',AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(i));
        end
    end
    fprintf('AvoMainRoVehInfo.d_veh2converge = %f\n',AvoMainRoVehInfo.d_veh2converge);
    fprintf('AvoMainRoVehInfo.d_veh2stopline = %f\n',AvoMainRoVehInfo.d_veh2stopline);
end
if CalibrationVars.UrbanPlanner.logTrigger(3)==1
    fprintf('AvoOncomingVehInfo.d_veh2waitingArea = %f\n',AvoOncomingVehInfo.d_veh2waitingArea);
    fprintf('AvoOncomingVehInfo.s_veh = \t');
    for i=1:length(AvoOncomingVehInfo.s_veh)
        if i==length(AvoOncomingVehInfo.s_veh)
            fprintf('%f\n',AvoOncomingVehInfo.s_veh(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.s_veh(i));
        end
    end
    fprintf('AvoOncomingVehInfo.v_veh = \t');
    for i=1:length(AvoOncomingVehInfo.v_veh)
        if i==length(AvoOncomingVehInfo.v_veh)
            fprintf('%f\n',AvoOncomingVehInfo.v_veh(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.v_veh(i));
        end
    end
    fprintf('AvoOncomingVehInfo.l_veh = \t');
    for i=1:length(AvoOncomingVehInfo.l_veh)
        if i==length(AvoOncomingVehInfo.l_veh)
            fprintf('%f\n',AvoOncomingVehInfo.l_veh(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.l_veh(i));
        end
    end
    fprintf('AvoOncomingVehInfo.d_veh2conflict = \t');
    for i=1:length(AvoOncomingVehInfo.d_veh2conflict)
        if i==length(AvoOncomingVehInfo.d_veh2conflict)
            fprintf('%f\n',AvoOncomingVehInfo.d_veh2conflict(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.d_veh2conflict(i));
        end
    end
    fprintf('AvoOncomingVehInfo.s_vehapostrophe = \t');
    for i=1:length(AvoOncomingVehInfo.s_vehapostrophe)
        if i==length(AvoOncomingVehInfo.s_vehapostrophe)
            fprintf('%f\n',AvoOncomingVehInfo.s_vehapostrophe(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.s_vehapostrophe(i));
        end
    end
    fprintf('AvoOncomingVehInfo.l_vehapostrophe = \t');
    for i=1:length(AvoOncomingVehInfo.l_vehapostrophe)
        if i==length(AvoOncomingVehInfo.l_vehapostrophe)
            fprintf('%f\n',AvoOncomingVehInfo.l_vehapostrophe(i));
        else
            fprintf('%f\t',AvoOncomingVehInfo.l_vehapostrophe(i));
        end
    end
end
if CalibrationVars.UrbanPlanner.logTrigger(4)==1
    fprintf('AvoPedInfo.d_veh2cross = %f\n',AvoPedInfo.d_veh2cross);
    fprintf('AvoPedInfo.w_cross = %f\n',AvoPedInfo.w_cross);
    fprintf('AvoPedInfo.s_ped = \t');
    for i=1:length(AvoPedInfo.s_ped)
        if i==length(AvoPedInfo.s_ped)
            fprintf('%f\n',AvoPedInfo.s_ped(i));
        else
            fprintf('%f\t',AvoPedInfo.s_ped(i));
        end
    end
    fprintf('AvoPedInfo.v_ped = \t');
    for i=1:length(AvoPedInfo.v_ped)
        if i==length(AvoPedInfo.v_ped)
            fprintf('%f\n',AvoPedInfo.v_ped(i));
        else
            fprintf('%f\t',AvoPedInfo.v_ped(i));
        end
    end
    fprintf('AvoPedInfo.l_ped = \t');
    for i=1:length(AvoPedInfo.l_ped)
        if i==length(AvoPedInfo.l_ped)
            fprintf('%f\n',AvoPedInfo.l_ped(i));
        else
            fprintf('%f\t',AvoPedInfo.l_ped(i));
        end
    end
    fprintf('AvoPedInfo.psi_ped = \t');
    for i=1:length(AvoPedInfo.psi_ped)
        if i==length(AvoPedInfo.psi_ped)
            fprintf('%f\n',AvoPedInfo.psi_ped(i));
        else
            fprintf('%f\t',AvoPedInfo.psi_ped(i));
        end
    end
end
if CalibrationVars.UrbanPlanner.logTrigger(5)==1
    fprintf('BasicsInfo.currentLaneFrontDis = %f\n',BasicsInfo.currentLaneFrontDis);
    fprintf('BasicsInfo.currentLaneFrontVel = %f\n',BasicsInfo.currentLaneFrontVel);
    fprintf('BasicsInfo.currentLaneFrontLen = %f\n',BasicsInfo.currentLaneFrontLen);
    fprintf('BasicsInfo.pos_s = %f\n',BasicsInfo.pos_s);
    fprintf('BasicsInfo.pos_l = %f\n',BasicsInfo.pos_l);
    fprintf('BasicsInfo.pos_psi = %f\n',BasicsInfo.pos_psi);
    fprintf('BasicsInfo.pos_l_CurrentLane = %f\n',BasicsInfo.pos_l_CurrentLane);
    fprintf('BasicsInfo.currentLaneIndex = %d\n',BasicsInfo.currentLaneIndex);
    fprintf('BasicsInfo.widthOfLanes = \t');
    for i=1:length(BasicsInfo.widthOfLanes)
        if i==length(BasicsInfo.widthOfLanes)
            fprintf('%f\n',BasicsInfo.widthOfLanes(i));
        else
            fprintf('%f\t',BasicsInfo.widthOfLanes(i));
        end
    end
    fprintf('BasicsInfo.targetLaneIndex = %d\n',BasicsInfo.targetLaneIndex);
    fprintf('BasicsInfo.v_max = %f\n',BasicsInfo.v_max);
    fprintf('BasicsInfo.goalLaneIndex = %d\n',BasicsInfo.goalLaneIndex);
    fprintf('BasicsInfo.d_veh2goal = %f\n',BasicsInfo.d_veh2goal);
    fprintf('BasicsInfo.sampleTime = %f\n',BasicsInfo.sampleTime);
end
if CalibrationVars.UrbanPlanner.logTrigger(6)==1
    fprintf('CalibrationVars.TrajPlanTurnAround.d_safe1 = %f\n',CalibrationVars.TrajPlanTurnAround.d_safe1);
    fprintf('CalibrationVars.TrajPlanTurnAround.d_safe2 = %f\n',CalibrationVars.TrajPlanTurnAround.d_safe2);
    fprintf('CalibrationVars.TrajPlanTurnAround.dec2line = %f\n',CalibrationVars.TrajPlanTurnAround.dec2line);
    fprintf('CalibrationVars.TrajPlanTurnAround.a_min = %f\n',CalibrationVars.TrajPlanTurnAround.a_min);
    fprintf('CalibrationVars.TrajPlanTurnAround.a_max_com = %f\n',CalibrationVars.TrajPlanTurnAround.a_max_com);
    fprintf('CalibrationVars.TrajPlanTurnAround.v_max_turnAround = %f\n',CalibrationVars.TrajPlanTurnAround.v_max_turnAround);
    fprintf('CalibrationVars.TrajPlanTurnAround.d_gap2stop = %f\n',CalibrationVars.TrajPlanTurnAround.d_gap2stop);
end
if CalibrationVars.UrbanPlanner.logTrigger(7)==1
    fprintf('CalibrationVars.SpeedPlanAvoidPedestrian.a_max = %f\n',CalibrationVars.SpeedPlanAvoidPedestrian.a_max);
    fprintf('CalibrationVars.SpeedPlanAvoidPedestrian.a_min = %f\n',CalibrationVars.SpeedPlanAvoidPedestrian.a_min);
    fprintf('CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int = %f\n',CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int);
    fprintf('CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg = %f\n',CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg);
    fprintf('CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped = %f\n',CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped);
end
if CalibrationVars.UrbanPlanner.logTrigger(8)==1
    fprintf('CalibrationVars.SpeedPlanTrafficLight.a_min_com = %f\n',CalibrationVars.SpeedPlanTrafficLight.a_min_com);
    fprintf('CalibrationVars.SpeedPlanTrafficLight.a_max = %f\n',CalibrationVars.SpeedPlanTrafficLight.a_max);
    fprintf('CalibrationVars.SpeedPlanTrafficLight.a_min = %f\n',CalibrationVars.SpeedPlanTrafficLight.a_min);
    fprintf('CalibrationVars.SpeedPlanTrafficLight.v_max_int = %f\n',CalibrationVars.SpeedPlanTrafficLight.v_max_int);
    fprintf('CalibrationVars.SpeedPlanTrafficLight.t_acc = %f\n',CalibrationVars.SpeedPlanTrafficLight.t_acc);
    fprintf('CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline = %f\n',CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline);
end
if CalibrationVars.UrbanPlanner.logTrigger(9)==1
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.a_min_com = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.a_min_com);
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.a_max = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.a_max);
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.a_min = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.a_min);
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.v_max = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.v_max);
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.t_re = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.t_re);
    fprintf('CalibrationVars.SpeedPlanAvoidVehicle.gapIndex = %f\n',CalibrationVars.SpeedPlanAvoidVehicle.gapIndex);
end
if CalibrationVars.UrbanPlanner.logTrigger(10)==1
    fprintf('CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com = %f\n',CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com);
    fprintf('CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min = %f\n',CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min);
    fprintf('CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int = %f\n',CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int);
    fprintf('CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe = %f\n',CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe);
end
if CalibrationVars.UrbanPlanner.logTrigger(11)==1
    fprintf('CalibrationVars.TrajPlanLaneChange.v_max_int = %f\n',CalibrationVars.TrajPlanLaneChange.v_max_int);
    fprintf('CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int = %f\n',CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int);
    fprintf('CalibrationVars.TrajPlanLaneChange.t_permit = %f\n',CalibrationVars.TrajPlanLaneChange.t_permit);
    fprintf('CalibrationVars.TrajPlanLaneChange.t_re = %f\n',CalibrationVars.TrajPlanLaneChange.t_re);
    fprintf('CalibrationVars.TrajPlanLaneChange.index_accel = %f\n',CalibrationVars.TrajPlanLaneChange.index_accel);
    fprintf('CalibrationVars.TrajPlanLaneChange.a_max_comfort = %f\n',CalibrationVars.TrajPlanLaneChange.a_max_comfort);
    fprintf('CalibrationVars.TrajPlanLaneChange.a_min = %f\n',CalibrationVars.TrajPlanLaneChange.a_min);
    fprintf('CalibrationVars.TrajPlanLaneChange.a_max = %f\n',CalibrationVars.TrajPlanLaneChange.a_max);
    fprintf('CalibrationVars.TrajPlanLaneChange.a_min_comfort = %f\n',CalibrationVars.TrajPlanLaneChange.a_min_comfort);
    fprintf('CalibrationVars.TrajPlanLaneChange.a_lateral = %f\n',CalibrationVars.TrajPlanLaneChange.a_lateral);
    fprintf('CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit = %f\n',CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit);
end
if CalibrationVars.UrbanPlanner.logTrigger(12)==1
    fprintf('CalibrationVars.ACC.a_max = %f\n',CalibrationVars.ACC.a_max);
    fprintf('CalibrationVars.ACC.a_min = %f\n',CalibrationVars.ACC.a_min);
    fprintf('CalibrationVars.ACC.d_wait2faultyCar = %f\n',CalibrationVars.ACC.d_wait2faultyCar);
    fprintf('CalibrationVars.ACC.tau_v_com = %f\n',CalibrationVars.ACC.tau_v_com);
    fprintf('CalibrationVars.ACC.tau_v = %f\n',CalibrationVars.ACC.tau_v);
    fprintf('CalibrationVars.ACC.tau_d = %f\n',CalibrationVars.ACC.tau_d);
    fprintf('CalibrationVars.ACC.tau_v_bre = %f\n',CalibrationVars.ACC.tau_v_bre);
    fprintf('CalibrationVars.ACC.tau_v_emg = %f\n',CalibrationVars.ACC.tau_v_emg);
    fprintf('CalibrationVars.ACC.tau_d_emg = %f\n',CalibrationVars.ACC.tau_d_emg);
    fprintf('CalibrationVars.ACC.t_acc = %f\n',CalibrationVars.ACC.t_acc);
    fprintf('CalibrationVars.ACC.d_wait = %f\n',CalibrationVars.ACC.d_wait);
end
if CalibrationVars.UrbanPlanner.logTrigger(13)==1
    fprintf('CalibrationVars.ACClowSpeed.a_max = %f\n',CalibrationVars.ACClowSpeed.a_max);
    fprintf('CalibrationVars.ACClowSpeed.a_min = %f\n',CalibrationVars.ACClowSpeed.a_min);
    fprintf('CalibrationVars.ACClowSpeed.a_min_com = %f\n',CalibrationVars.ACClowSpeed.a_min_com);
    fprintf('CalibrationVars.ACClowSpeed.tau_v_com = %f\n',CalibrationVars.ACClowSpeed.tau_v_com);
    fprintf('CalibrationVars.ACClowSpeed.tau_v = %f\n',CalibrationVars.ACClowSpeed.tau_v);
    fprintf('CalibrationVars.ACClowSpeed.tau_d = %f\n',CalibrationVars.ACClowSpeed.tau_d);
    fprintf('CalibrationVars.ACClowSpeed.tau_v_bre = %f\n',CalibrationVars.ACClowSpeed.tau_v_bre);
    fprintf('CalibrationVars.ACClowSpeed.tau_v_emg = %f\n',CalibrationVars.ACClowSpeed.tau_v_emg);
    fprintf('CalibrationVars.ACClowSpeed.tau_d_emg = %f\n',CalibrationVars.ACClowSpeed.tau_d_emg);
    fprintf('CalibrationVars.ACClowSpeed.tau_d_lowspeed = %f\n',CalibrationVars.ACClowSpeed.tau_d_lowspeed);
    fprintf('CalibrationVars.ACClowSpeed.t_acc = %f\n',CalibrationVars.ACClowSpeed.t_acc);
    fprintf('CalibrationVars.ACClowSpeed.d_wait = %f\n',CalibrationVars.ACClowSpeed.d_wait);
end
if CalibrationVars.UrbanPlanner.logTrigger(14)==1
    fprintf('CalibrationVars.Decider.a_bre = %f\n',CalibrationVars.Decider.a_bre);
    fprintf('CalibrationVars.Decider.a_bre_com = %f\n',CalibrationVars.Decider.a_bre_com);
    fprintf('CalibrationVars.Decider.idle_speed = %f\n',CalibrationVars.Decider.idle_speed);
    fprintf('CalibrationVars.Decider.dist_wait2pilot = %f\n',CalibrationVars.Decider.dist_wait2pilot);
    fprintf('CalibrationVars.Decider.dist_wait2veh = %f\n',CalibrationVars.Decider.dist_wait2veh);
    fprintf('CalibrationVars.Decider.glosaAdp = %f\n',CalibrationVars.Decider.glosaAdp);
    fprintf('CalibrationVars.Decider.mrg = %f\n',CalibrationVars.Decider.mrg);
    fprintf('CalibrationVars.Decider.desRate = %f\n',CalibrationVars.Decider.desRate);
    fprintf('CalibrationVars.Decider.dIntxn = %f\n',CalibrationVars.Decider.dIntxn);
    fprintf('CalibrationVars.Decider.dMin = %f\n',CalibrationVars.Decider.dMin);
    fprintf('CalibrationVars.Decider.dec = %f\n',CalibrationVars.Decider.dec);
    fprintf('CalibrationVars.Decider.glosaAverageIndex = %f\n',CalibrationVars.Decider.glosaAverageIndex);
    fprintf('CalibrationVars.Decider.d_veh2endpoint = %f\n',CalibrationVars.Decider.d_veh2endpoint);
    fprintf('CalibrationVars.Decider.glosaVMin = %f\n',CalibrationVars.Decider.glosaVMin);
    fprintf('CalibrationVars.UrbanPlanner.jerkLimit = %f\n',CalibrationVars.UrbanPlanner.jerkLimit);
end
if CalibrationVars.UrbanPlanner.logTrigger(15)==1
    fprintf('CalibrationVars.AEBDecision.minGapIsTolerated = %f\n',CalibrationVars.AEBDecision.minGapIsTolerated);
end
if CalibrationVars.UrbanPlanner.logTrigger(16)==1
    fprintf('ChassisInfo.speed = %f\n',ChassisInfo.speed);
    fprintf('ChassisInfo.currentGear = %d\n',ChassisInfo.currentGear);
end
if CalibrationVars.UrbanPlanner.logTrigger(17)==1
    fprintf('GlobVars.AEBDecision.AEBActive = %d\n',GlobVars.AEBDecision.AEBActive);
end
if CalibrationVars.UrbanPlanner.logTrigger(18)==1
    fprintf('GlobVars.TrajPlanTurnAround.posCircle = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.posCircle)
        if i==length(GlobVars.TrajPlanTurnAround.posCircle)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.posCircle(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.posCircle(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.posCircle2 = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.posCircle2)
        if i==length(GlobVars.TrajPlanTurnAround.posCircle2)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.posCircle2(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.posCircle2(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.posCircle3 = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.posCircle3)
        if i==length(GlobVars.TrajPlanTurnAround.posCircle3)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.posCircle3(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.posCircle3(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_start = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_start)
        if i==length(GlobVars.TrajPlanTurnAround.pos_start)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_start(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_start(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_mid1 = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_mid1)
        if i==length(GlobVars.TrajPlanTurnAround.pos_mid1)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_mid1(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_mid1(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_mid2 = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_mid2)
        if i==length(GlobVars.TrajPlanTurnAround.pos_mid2)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_mid2(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_mid2(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_mid1_rear = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_mid1_rear)
        if i==length(GlobVars.TrajPlanTurnAround.pos_mid1_rear)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_mid1_rear(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_mid1_rear(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_mid2_rear = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_mid2_rear)
        if i==length(GlobVars.TrajPlanTurnAround.pos_mid2_rear)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_mid2_rear(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_mid2_rear(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.pos_end = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.pos_end)
        if i==length(GlobVars.TrajPlanTurnAround.pos_end)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.pos_end(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.pos_end(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.laneCenterline = \t');
    for i=1:length(GlobVars.TrajPlanTurnAround.laneCenterline)
        if i==length(GlobVars.TrajPlanTurnAround.laneCenterline)
            fprintf('%f\n',GlobVars.TrajPlanTurnAround.laneCenterline(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanTurnAround.laneCenterline(i));
        end
    end
    fprintf('GlobVars.TrajPlanTurnAround.dec_trunAround = %d\n',GlobVars.TrajPlanTurnAround.dec_trunAround);
    fprintf('GlobVars.TrajPlanTurnAround.wait_turnAround = %d\n',GlobVars.TrajPlanTurnAround.wait_turnAround);
    fprintf('GlobVars.TrajPlanTurnAround.typeOfTurnAround = %d\n',GlobVars.TrajPlanTurnAround.typeOfTurnAround);
    fprintf('GlobVars.TrajPlanTurnAround.turnAroundState = %d\n',GlobVars.TrajPlanTurnAround.turnAroundState);
    fprintf('GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite = %d\n',GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite);
    fprintf('GlobVars.TrajPlanTurnAround.turnAroundActive = %d\n',GlobVars.TrajPlanTurnAround.turnAroundActive);
    fprintf('GlobVars.TrajPlanTurnAround.reflineSend = %f\n',GlobVars.TrajPlanTurnAround.reflineSend);
    fprintf('GlobVars.TrajPlanTurnAround.reflineLend = %f\n',GlobVars.TrajPlanTurnAround.reflineLend);
end
if CalibrationVars.UrbanPlanner.logTrigger(19)==1
    fprintf('GlobVars.SpeedPlanAvoidPedestrian.dec_ped = %d\n',GlobVars.SpeedPlanAvoidPedestrian.dec_ped);
    fprintf('GlobVars.SpeedPlanAvoidPedestrian.wait_ped = %d\n',GlobVars.SpeedPlanAvoidPedestrian.wait_ped);
    fprintf('GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight = %d\n',GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight);
    fprintf('GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight = %d\n',GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight);
    fprintf('GlobVars.SpeedPlanTrafficLight.wait_TrafficLight = %d\n',GlobVars.SpeedPlanTrafficLight.wait_TrafficLight);
    fprintf('GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle = %d\n',GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle);
    fprintf('GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle = %d\n',GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle);
    fprintf('GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle = %d\n',GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle);
    fprintf('GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle = %d\n',GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle);
    fprintf('GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle = %d\n',GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle);
end
if CalibrationVars.UrbanPlanner.logTrigger(20)==1
    fprintf('GlobVars.TrajPlanLaneChange.countLaneChange = %d\n',GlobVars.TrajPlanLaneChange.countLaneChange);
    fprintf('GlobVars.TrajPlanLaneChange.durationLaneChange = %d\n',GlobVars.TrajPlanLaneChange.durationLaneChange);
    fprintf('GlobVars.TrajPlanLaneChange.laneChangePath = \t');
    for i=1:length(GlobVars.TrajPlanLaneChange.laneChangePath)
        if i==length(GlobVars.TrajPlanLaneChange.laneChangePath)
            fprintf('%f\n',GlobVars.TrajPlanLaneChange.laneChangePath(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanLaneChange.laneChangePath(i));
        end
    end
    fprintf('GlobVars.TrajPlanLaneChange.t_lc_traj = %f\n',GlobVars.TrajPlanLaneChange.t_lc_traj);
    fprintf('GlobVars.TrajPlanLaneChange.currentTargetLaneIndex = %d\n',GlobVars.TrajPlanLaneChange.currentTargetLaneIndex);
end
if CalibrationVars.UrbanPlanner.logTrigger(21)==1
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan = %d\n',GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan);
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.para = \t');
    for i=1:length(GlobVars.TrajPlanLaneChange_RePlan.para)
        if i==length(GlobVars.TrajPlanLaneChange_RePlan.para)
            fprintf('%f\n',GlobVars.TrajPlanLaneChange_RePlan.para(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanLaneChange_RePlan.para(i));
        end
    end
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.s_end = %f\n',GlobVars.TrajPlanLaneChange_RePlan.s_end);
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.l_end = %f\n',GlobVars.TrajPlanLaneChange_RePlan.l_end);
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.para1 = \t');
    for i=1:length(GlobVars.TrajPlanLaneChange_RePlan.para1)
        if i==length(GlobVars.TrajPlanLaneChange_RePlan.para1)
            fprintf('%f\n',GlobVars.TrajPlanLaneChange_RePlan.para1(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanLaneChange_RePlan.para1(i));
        end
    end
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.para2 = \t');
    for i=1:length(GlobVars.TrajPlanLaneChange_RePlan.para2)
        if i==length(GlobVars.TrajPlanLaneChange_RePlan.para2)
            fprintf('%f\n',GlobVars.TrajPlanLaneChange_RePlan.para2(i));
        else
            fprintf('%f\t',GlobVars.TrajPlanLaneChange_RePlan.para2(i));
        end
    end
    fprintf('GlobVars.TrajPlanLaneChange_RePlan.para3 = %f\n',GlobVars.TrajPlanLaneChange_RePlan.para3);
end
if CalibrationVars.UrbanPlanner.logTrigger(22)==1
    fprintf('GlobVars.Decider.dec_start = %d\n',GlobVars.Decider.dec_start);
    fprintf('GlobVars.Decider.dir_start = %d\n',GlobVars.Decider.dir_start);
    fprintf('GlobVars.Decider.countLaneChangeDecider = %d\n',GlobVars.Decider.countLaneChangeDecider);
    fprintf('GlobVars.Decider.currentTargetLaneIndexDecider = %d\n',GlobVars.Decider.currentTargetLaneIndexDecider);
    fprintf('GlobVars.Decider.a_soll_pre = %f\n',GlobVars.Decider.a_soll_pre);
    fprintf('GlobVars.Decider.a_sollpre2traj = %f\n',GlobVars.Decider.a_sollpre2traj);
    fprintf('GlobVars.Decider.wait_pullover = %d\n',GlobVars.Decider.wait_pullover);
    fprintf('GlobVars.Decider.distBehindGoal = %f\n',GlobVars.Decider.distBehindGoal);
    fprintf('GlobVars.Decider.dec_follow = %d\n',GlobVars.Decider.dec_follow);
end
if CalibrationVars.UrbanPlanner.logTrigger(23)==1
    fprintf('GlobVars.SpeedPlanStopSign.wait_stopsign = %d\n',GlobVars.SpeedPlanStopSign.wait_stopsign);
end
if CalibrationVars.UrbanPlanner.logTrigger(29)==1
    fprintf('GlosaActive = %d\n',GlosaActive);
end
if CalibrationVars.UrbanPlanner.logTrigger(24)==1
    fprintf('LaneChangeActive = %d\n',LaneChangeActive);
    fprintf('LaneChangeInfo.d_veh2int = %f\n',LaneChangeInfo.d_veh2int);
    fprintf('LaneChangeInfo.leftLaneBehindDis = %f\n',LaneChangeInfo.leftLaneBehindDis);
    fprintf('LaneChangeInfo.leftLaneBehindVel = %f\n',LaneChangeInfo.leftLaneBehindVel);
    fprintf('LaneChangeInfo.leftLaneFrontDis = %f\n',LaneChangeInfo.leftLaneFrontDis);
    fprintf('LaneChangeInfo.leftLaneFrontVel = %f\n',LaneChangeInfo.leftLaneFrontVel);
    fprintf('LaneChangeInfo.rightLaneBehindDis = %f\n',LaneChangeInfo.rightLaneBehindDis);
    fprintf('LaneChangeInfo.rightLaneBehindVel = %f\n',LaneChangeInfo.rightLaneBehindVel);
    fprintf('LaneChangeInfo.rightLaneFrontDis = %f\n',LaneChangeInfo.rightLaneFrontDis);
    fprintf('LaneChangeInfo.rightLaneFrontVel = %f\n',LaneChangeInfo.rightLaneFrontVel);
    fprintf('LaneChangeInfo.leftLaneBehindLen = %f\n',LaneChangeInfo.leftLaneBehindLen);
    fprintf('LaneChangeInfo.leftLaneFrontLen = %f\n',LaneChangeInfo.leftLaneFrontLen);
    fprintf('LaneChangeInfo.rightLaneBehindLen = %f\n',LaneChangeInfo.rightLaneBehindLen);
    fprintf('LaneChangeInfo.rightLaneFrontLen = %f\n',LaneChangeInfo.rightLaneFrontLen);
end
if CalibrationVars.UrbanPlanner.logTrigger(25)==1
    fprintf('Parameters.turningRadius = %f\n',Parameters.turningRadius);
    fprintf('Parameters.w_veh = %f\n',Parameters.w_veh);
    fprintf('Parameters.l_veh = %f\n',Parameters.l_veh);
end
if CalibrationVars.UrbanPlanner.logTrigger(29)==1
    fprintf('PedestrianActive = %d\n',PedestrianActive);
    fprintf('PlannerLevel = %d\n',PlannerLevel);
end
if CalibrationVars.UrbanPlanner.logTrigger(26)==1
    fprintf('StopSignInfo.d_veh2stopline = %f\n',StopSignInfo.d_veh2stopline);
end
if CalibrationVars.UrbanPlanner.logTrigger(29)==1
    fprintf('TrafficLightActive = %d\n',TrafficLightActive);
end
if CalibrationVars.UrbanPlanner.logTrigger(27)==1
    fprintf('TrafficLightInfo.greenLight = %f\n',TrafficLightInfo.greenLight);
    fprintf('TrafficLightInfo.d_veh2stopline = %f\n',TrafficLightInfo.d_veh2stopline);
    fprintf('TrafficLightInfo.phase = \t');
    for i=1:length(TrafficLightInfo.phase)
        if i==length(TrafficLightInfo.phase)
            fprintf('%f\n',TrafficLightInfo.phase(i));
        else
            fprintf('%f\t',TrafficLightInfo.phase(i));
        end
    end
end
if CalibrationVars.UrbanPlanner.logTrigger(29)==1
    fprintf('TurnAroundActive = %d\n',TurnAroundActive);
end
if CalibrationVars.UrbanPlanner.logTrigger(28)==1
    fprintf('TurnAroundInfo.numOfLanesOpposite = %d\n',TurnAroundInfo.numOfLanesOpposite);
    fprintf('TurnAroundInfo.widthOfLanesOpposite = \t');
    for i=1:length(TurnAroundInfo.widthOfLanesOpposite)
        if i==length(TurnAroundInfo.widthOfLanesOpposite)
            fprintf('%f\n',TurnAroundInfo.widthOfLanesOpposite(i));
        else
            fprintf('%f\t',TurnAroundInfo.widthOfLanesOpposite(i));
        end
    end
    fprintf('TurnAroundInfo.widthOfGap = %f\n',TurnAroundInfo.widthOfGap);
    fprintf('TurnAroundInfo.s_turnaround_border = %f\n',TurnAroundInfo.s_turnaround_border);
    fprintf('TurnAroundInfo.indexOfLaneOppositeCar= \t');
    for i=1:length(TurnAroundInfo.indexOfLaneOppositeCar)
        if i==length(TurnAroundInfo.indexOfLaneOppositeCar)
            fprintf('%d\n',TurnAroundInfo.indexOfLaneOppositeCar(i));
        else
            fprintf('%d\t',TurnAroundInfo.indexOfLaneOppositeCar(i));
        end
    end
    fprintf('TurnAroundInfo.speedOppositeCar = \t');
    for i=1:length(TurnAroundInfo.speedOppositeCar)
        if i==length(TurnAroundInfo.speedOppositeCar)
            fprintf('%f\n',TurnAroundInfo.speedOppositeCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.speedOppositeCar(i));
        end
    end
    fprintf('TurnAroundInfo.posSOppositeCar = \t');
    for i=1:length(TurnAroundInfo.posSOppositeCar)
        if i==length(TurnAroundInfo.posSOppositeCar)
            fprintf('%f\n',TurnAroundInfo.posSOppositeCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.posSOppositeCar(i));
        end
    end
    fprintf('TurnAroundInfo.indexOfLaneCodirectCar= \t');
    for i=1:length(TurnAroundInfo.indexOfLaneCodirectCar)
        if i==length(TurnAroundInfo.indexOfLaneCodirectCar)
            fprintf('%d\n',TurnAroundInfo.indexOfLaneCodirectCar(i));
        else
            fprintf('%d\t',TurnAroundInfo.indexOfLaneCodirectCar(i));
        end
    end
    fprintf('TurnAroundInfo.speedCodirectCar = \t');
    for i=1:length(TurnAroundInfo.speedCodirectCar)
        if i==length(TurnAroundInfo.speedCodirectCar)
            fprintf('%f\n',TurnAroundInfo.speedCodirectCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.speedCodirectCar(i));
        end
    end
    fprintf('TurnAroundInfo.posSCodirectCar = \t');
    for i=1:length(TurnAroundInfo.posSCodirectCar)
        if i==length(TurnAroundInfo.posSCodirectCar)
            fprintf('%f\n',TurnAroundInfo.posSCodirectCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.posSCodirectCar(i));
        end
    end
    fprintf('TurnAroundInfo.lengthOppositeCar = \t');
    for i=1:length(TurnAroundInfo.lengthOppositeCar)
        if i==length(TurnAroundInfo.lengthOppositeCar)
            fprintf('%f\n',TurnAroundInfo.lengthOppositeCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.lengthOppositeCar(i));
        end
    end
    fprintf('TurnAroundInfo.lengthCodirectCar = \t');
    for i=1:length(TurnAroundInfo.lengthCodirectCar)
        if i==length(TurnAroundInfo.lengthCodirectCar)
            fprintf('%f\n',TurnAroundInfo.lengthCodirectCar(i));
        else
            fprintf('%f\t',TurnAroundInfo.lengthCodirectCar(i));
        end
    end
end
if CalibrationVars.UrbanPlanner.logTrigger(29)==1
    fprintf('VehicleCrossingActive = %d\n',VehicleCrossingActive);
    fprintf('VehicleOncomingActive = %d\n',VehicleOncomingActive);
end
end

function logOutput(Trajectory,Decision,Refline,CalibrationVars)
if CalibrationVars.UrbanPlanner.logTrigger(30)==1
    fprintf('Decision.AEBactive = %d\n',Decision.AEBactive);
    fprintf('Decision.TargetGear = %d\n',Decision.TargetGear);
    fprintf('Decision.states = %d\n',Decision.states);
    fprintf('Decision.LaneChange = %d\n',Decision.LaneChange);
    fprintf('Decision.SlowDown = %d\n',Decision.SlowDown);
    fprintf('Decision.TargetSpeed = %f\n',Decision.TargetSpeed);
    fprintf('Decision.Wait = %d\n',Decision.Wait);
    fprintf('Decision.WaitDistance = %f\n',Decision.WaitDistance);
    fprintf('Decision.Start = %d\n',Decision.Start);
    fprintf('Decision.a_soll = %f\n',Decision.a_soll);
    fprintf('Decision.PedestrianState = %d\n',Decision.PedestrianState);
    fprintf('Decision.TrafficLightState = %d\n',Decision.TrafficLightState);
    fprintf('Decision.VehicleCrossingState = %d\n',Decision.VehicleCrossingState);
    fprintf('Decision.VehicleOncomingState = %d\n',Decision.VehicleOncomingState);
    fprintf('Decision.StopSignState = %d\n',Decision.StopSignState);
    fprintf('Decision.FollowState = %d\n',Decision.FollowState);
    fprintf('Decision.PullOverState = %d\n',Decision.PullOverState);
    fprintf('Decision.TurnAroundState = %d\n',Decision.TurnAroundState);
end
if CalibrationVars.UrbanPlanner.logTrigger(31)==1
    fprintf('Refline.NumRefLaneTurnAround = %d\n',Refline.NumRefLaneTurnAround);
    fprintf('Refline.SRefLaneTurnAround = \t');
    for i=1:length(Refline.SRefLaneTurnAround)
        if i==length(Refline.SRefLaneTurnAround)
            fprintf('%f\n',Refline.SRefLaneTurnAround(i));
        else
            fprintf('%f\t',Refline.SRefLaneTurnAround(i));
        end
    end
    fprintf('Refline.LRefLaneTurnAround = \t');
    for i=1:length(Refline.LRefLaneTurnAround)
        if i==length(Refline.LRefLaneTurnAround)
            fprintf('%f\n',Refline.LRefLaneTurnAround(i));
        else
            fprintf('%f\t',Refline.LRefLaneTurnAround(i));
        end
    end
    fprintf('Refline.TurnAroundReflineState = %d\n',Refline.TurnAroundReflineState);
end
if CalibrationVars.UrbanPlanner.logTrigger(32)==1
    fprintf('Trajectory.traj_s = \t');
    for i=1:length(Trajectory.traj_s)
        if i==length(Trajectory.traj_s)
            fprintf('%f\n',Trajectory.traj_s(i));
        else
            fprintf('%f\t',Trajectory.traj_s(i));
        end
    end
    fprintf('Trajectory.traj_l = \t');
    for i=1:length(Trajectory.traj_l)
        if i==length(Trajectory.traj_l)
            fprintf('%f\n',Trajectory.traj_l(i));
        else
            fprintf('%f\t',Trajectory.traj_l(i));
        end
    end
    fprintf('Trajectory.traj_psi = \t');
    for i=1:length(Trajectory.traj_psi)
        if i==length(Trajectory.traj_psi)
            fprintf('%f\n',Trajectory.traj_psi(i));
        else
            fprintf('%f\t',Trajectory.traj_psi(i));
        end
    end
    fprintf('Trajectory.traj_vs = \t');
    for i=1:length(Trajectory.traj_vs)
        if i==length(Trajectory.traj_vs)
            fprintf('%f\n',Trajectory.traj_vs(i));
        else
            fprintf('%f\t',Trajectory.traj_vs(i));
        end
    end
    fprintf('Trajectory.traj_vl = \t');
    for i=1:length(Trajectory.traj_vl)
        if i==length(Trajectory.traj_vl)
            fprintf('%f\n',Trajectory.traj_vl(i));
        else
            fprintf('%f\t',Trajectory.traj_vl(i));
        end
    end
    fprintf('Trajectory.traj_omega = \t');
    for i=1:length(Trajectory.traj_omega)
        if i==length(Trajectory.traj_omega)
            fprintf('%f\n',Trajectory.traj_omega(i));
        else
            fprintf('%f\t',Trajectory.traj_omega(i));
        end
    end
    fprintf('Trajectory.planning_states = %d\n',Trajectory.planning_states);
end
end