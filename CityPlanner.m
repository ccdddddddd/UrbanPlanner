clearvars -except pathcase pathID;
close('all');
clc;
%% 加载标定表和net
load('calibration.mat')
Net=xmlread('City.net.xml');
tlLogicItems = Net.getElementsByTagName('tlLogic');
NumofElements=tlLogicItems.getLength;
if NumofElements~=0
    tlLogic{NumofElements,2}=[];
    for item=1:NumofElements
        tlLogic(item,1)=tlLogicItems.item(item-1).getAttribute('id');
        tlLogic(item,2)=tlLogicItems.item(item-1).getAttribute('offset');
    end
    % tlLogic(1,1)=tlLogicItems.item(0).getAttribute('id');
    % tlLogic(1,2)=tlLogicItems.item(0).getAttribute('offset');
    TrafficLightOffset=str2double(tlLogic(strcmp(tlLogic(:,1),{'8'}),2));
    TrafficLightOffset2=str2double(tlLogic(strcmp(tlLogic(:,1),{'11'}),2));
    TrafficLightOffset3=str2double(tlLogic(strcmp(tlLogic(:,1),{'gneJ0'}),2));
    TrafficLightOffset4=str2double(tlLogic(strcmp(tlLogic(:,1),{'9'}),2));
    TrafficLightOffset5=str2double(tlLogic(strcmp(tlLogic(:,1),{'J12'}),2));
    TrafficLightOffset6=str2double(tlLogic(strcmp(tlLogic(:,1),{'J6'}),2));
end
%% 初始化全局变量
% 设置参数
import traci.constants;
v_max=50/3.6;
NumOfLanes=1;
% TurningRadius=5;
%基本参数
Parameters.TurningRadius=double(TurningRadius);%20220323
Parameters.w_veh=double(1.8);
Parameters.l_veh=double(5);
%标定量
CalibrationVars.TrajPlanTurnAround.D_safe1=0.5;
CalibrationVars.TrajPlanTurnAround.D_safe2=2;
CalibrationVars.TrajPlanTurnAround.dec2line=0.2;
CalibrationVars.TrajPlanTurnAround.a_min=-2;
CalibrationVars.TrajPlanTurnAround.a_max_com=1.5;
CalibrationVars.TrajPlanTurnAround.v_max_turnAround=5;
CalibrationVars.SpeedPlanAvoidPedestrian.a_max=2.5;
CalibrationVars.SpeedPlanAvoidPedestrian.a_min=-3;
CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int=30/3.6;
CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg=20/3.6;
CalibrationVars.SpeedPlanTrafficLight.a_min_com=-1.5;
CalibrationVars.SpeedPlanTrafficLight.a_max=2.5;
CalibrationVars.SpeedPlanTrafficLight.a_min=-3;
CalibrationVars.SpeedPlanTrafficLight.v_max=14;
CalibrationVars.SpeedPlanTrafficLight.v_max_int=30/3.6;
CalibrationVars.SpeedPlanTrafficLight.t_acc=1.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_min_com=-1.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_max=1.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_min=-3;
CalibrationVars.SpeedPlanAvoidVehicle.v_max=40/3.6;
CalibrationVars.SpeedPlanAvoidVehicle.t_re=1.5;
CalibrationVars.SpeedPlanAvoidVehicle.GapIndex=2;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com=1.5;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min=-3;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int=30/3.6;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.D_safe=2;
CalibrationVars.TrajPlanLaneChange.v_max_int=30/3.6;
CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int=1;
CalibrationVars.TrajPlanLaneChange.t_permit=3;
CalibrationVars.TrajPlanLaneChange.t_re=0.5;
CalibrationVars.TrajPlanLaneChange.index_accel=0.5;
CalibrationVars.TrajPlanLaneChange.a_max_comfort=1;
CalibrationVars.TrajPlanLaneChange.a_min=-3.5;
CalibrationVars.TrajPlanLaneChange.a_max=2.5;
CalibrationVars.TrajPlanLaneChange.a_min_comfort=-1;
CalibrationVars.TrajPlanLaneChange_RePlan.t_re=0.5;
CalibrationVars.TrajPlanLaneChange_RePlan.index_accel=0.5;
CalibrationVars.TrajPlanLaneChange_RePlan.a_min_comfort=-1;
CalibrationVars.TrajPlanLaneChange_RePlan.a_min=-3.5;
CalibrationVars.ACC.a_max=2.5;
CalibrationVars.ACC.a_min=-4;
CalibrationVars.ACC.a_min_com=-1.5;
CalibrationVars.ACC.tau_v_com=4;
CalibrationVars.ACC.tau_v=2;
CalibrationVars.ACC.tau_d=5;
CalibrationVars.ACC.tau_v_bre=1;
CalibrationVars.ACC.tau_v_emg=0.5;
CalibrationVars.ACC.tau_d_emg=2;
CalibrationVars.ACC.t_acc=2;
CalibrationVars.ACCcust.tau_v_com=4;
CalibrationVars.ACCcust.tau_v=2;
CalibrationVars.ACCcust.tau_d=5;
CalibrationVars.ACClowSpeed.a_max=2.5;
CalibrationVars.ACClowSpeed.a_min=-4;
CalibrationVars.ACClowSpeed.a_min_com=-1.5;
CalibrationVars.ACClowSpeed.tau_v_com=4;
CalibrationVars.ACClowSpeed.tau_v=2;
CalibrationVars.ACClowSpeed.tau_d=5;
CalibrationVars.ACClowSpeed.tau_v_bre=1;
CalibrationVars.ACClowSpeed.tau_v_emg=0.5;
CalibrationVars.ACClowSpeed.tau_d_emg=2;
CalibrationVars.ACClowSpeed.tau_d_lowspeed=5/2;
CalibrationVars.ACClowSpeed.t_acc=2;
CalibrationVars.ACC.d_wait=4;
CalibrationVars.ACClowSpeed.d_wait=4;%4
% 全局变量
GlobVars.AEBDecision.AEBActive=int16(0);
GlobVars.TrajPlanTurnAround.PosCircle=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.PosCircle2=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.PosCircle3=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.pos_start=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.pos_mid1=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid2=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid1_rear=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid2_rear=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_end=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.LaneCenterline=zeros(1,7,'double');
GlobVars.TrajPlanTurnAround.dec_trunAround=int16(0);
GlobVars.TrajPlanTurnAround.wait_turnAround=int16(0);
GlobVars.TrajPlanTurnAround.TypeOfTurnAround=int16(0);
GlobVars.TrajPlanTurnAround.TurnAroundState=int16(0);
GlobVars.TrajPlanTurnAround.TargetLaneIndexOpposite=int16(0);
GlobVars.TrajPlanTurnAround.TurnAroundActive=int16(0);
GlobVars.SpeedPlanAvoidPedestrian.dec_ped=int16(0);
GlobVars.SpeedPlanAvoidPedestrian.wait_ped=int16(0);
GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight=int16(0);
GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight=int16(0);
GlobVars.SpeedPlanTrafficLight.wait_TrafficLight=int16(0);
GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle=int16(0);
GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle=int16(0);
GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle=int16(0);
GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle=int16(0);
GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle=int16(0);
GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0);
GlobVars.TrajPlanLaneChange.DurationLaneChange=int16(0);
GlobVars.TrajPlanLaneChange.LaneChangePath=zeros([6/0.05 6],'double');
GlobVars.TrajPlanLaneChange.t_lc_traj=double(2);
GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex=int16(0);
GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan=int16(0);
GlobVars.TrajPlanLaneChange_RePlan.LaneChangePath_RePlan=zeros([6/0.05 6],'double');
GlobVars.TrajPlanLaneChange_RePlan.t_lc_RePlan=double(0);
GlobVars.Decider.dec_start=int16(0);
GlobVars.Decider.dir_start=int16(0);
GlobVars.Decider.CountLaneChangeDecider=int16(0);
GlobVars.Decider.CurrentTargetLaneIndexDecider=int16(0);
GlobVars.SpeedPlanStopSign.wait_stopsign=int16(0);
Decision.TargetGear=int16(0);
CurrentGear=0;
RefLaneIndex=0;
TimeResponse=0;
TimeResponse1=0;
TimeResponse2=0;
AEBdelay=0;
StopVeh=0;
 Fllowsts=0;
 manual=0;%0,轨迹规划，1、按键触发，2、决策结果、3、文字输出
 global_route={};
%% 设置sumo路径
if usecase<=31||(usecase>=168&&usecase<=173)
    path=strcat('Sumocfg/TurnAround/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase<=33&&usecase>31
    path=strcat('Sumocfg/01_TurnLeftOfY/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase<=36&&usecase>33
    path=strcat('Sumocfg/02_AvoidPedestrian/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase<=41&&usecase>36
    path=strcat('Sumocfg/03_LanesConverge/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase>=42&&usecase<=46)||(usecase>=156&&usecase<=161)
    path=strcat('Sumocfg/04_LaneChangeDecisions/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase<=48&&usecase>46)||(usecase>=167&&usecase<=167)
    path=strcat('Sumocfg/05_TurnRightOfIntersection/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=49&&usecase<=54
    path=strcat('Sumocfg/06_CrossSidewalk/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=55&&usecase<=60
    path=strcat('Sumocfg/07_TrafficLightDecisions/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=61&&usecase<=63
    path=strcat('Sumocfg/08_AbortLaneChange/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase>=64&&usecase<=69)||(usecase>=165&&usecase<=166)
    path=strcat('Sumocfg/09_RampIn/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase>=70&&usecase<=75)||(usecase>=153&&usecase<=155)
    path=strcat('Sumocfg/10_TurnLeftOfIntersection/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=100&&usecase<=100
    path=strcat('Sumocfg/11_integration/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=76&&usecase<=77
    path=strcat('Sumocfg/12_StraightOfIntersection/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=78&&usecase<=97
    path=strcat('Sumocfg/13_AvoidFaultyCars/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=101&&usecase<=103
    path=strcat('Sumocfg/15_RoundAbout/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=104&&usecase<=114
    path=strcat('Sumocfg/16_NotrafficlightIntersections/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=201&&usecase<=211
    path=strcat('Sumocfg/20_Decider_LaneChange/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase>=115&&usecase<=125)||(usecase>=162&&usecase<=164)
    path=strcat('Sumocfg/03_LanesConverge/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start')); 
elseif usecase>=126&&usecase<=130
    path=strcat('Sumocfg/06_CrossSidewalk/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));  
elseif usecase>=131&&usecase<=131
    path=strcat('Sumocfg/02_AvoidPedestrian/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=212&&usecase<=217
    path=strcat('Sumocfg/21_Decider_SlowDown/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=218&&usecase<=231
    path=strcat('Sumocfg/22_Decider_Wait_Start/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=132&&usecase<=144
    path=strcat('Sumocfg/17_PullOver/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=145&&usecase<=152
    path=strcat('Sumocfg/18_StopSign/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=232&&usecase<=241
    path=strcat('Sumocfg/23_Decider_AEB/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
else
    traci.start('sumo-gui -c ./City.sumocfg --start');
end
% Launch SUMO in server mode and initialize the TraCI connection
%  traci.start('sumo-gui -c ./City.sumocfg --start');
%% 设置仿真时间
SIM_STEPS = [1 4560];
beginTime = SIM_STEPS(1);
duration =  SIM_STEPS(2);
endTime =  SIM_STEPS(1) +  SIM_STEPS(2) - 1;
% phase = zeros(1,duration);
% traci.vehicle.setColor('S0',[0 255 255 200]);
%%
delay=0;
%画图
if manual==2||(usecase>=201&&usecase<=205)
    t=linspace(0,(duration-1)/10,duration);
    w=zeros(1,duration)*nan;
%     w(:,700)=[0];
    w(1,700)=0;
    p=plot(t,w,'.-');
    x=45;
    axis([x x+50 -0.5 3]);
    grid on;
    label=legend('Decision.LaneChange');
    set(gcf,'Position',[1,40,540,290])
    get(gcf,'Position');
    g=text(x+12,2.5,['speed' 32 '=' 32 num2str(0) 'km/h']);
    wait_text=text(x+1,2.7,['wait' 32 '=' 32 num2str(0)]);
    waitDist_text=text(x+1,2.5,['dist' 32 32 '=' 32 num2str(0) 'm']);
    slowdown_text=text(x+12,2.9,['slowd' 32 32 '=' 32 num2str(0)]);
    targetspeed_text=text(x+12,2.7,['targV' 32 32 '=' 32 num2str(0) 'km/h']);
    start_text=text(x+1,2.9,['start' 32 '=' 32 num2str(0)]);
    AEB_text=text(x+22,2.5,['AEB' 32 '=' 32 num2str(0)]);
    Title=title(['TargetLane=' num2str(0) 32 32 32 32 'CurrentLane=' num2str(0)]);
    xlabel('t')
    yticklabels({'Keep','Left','Right'});
    yticks(0:1:2);
    drawnow
elseif manual==3
    h=figure;
    ax=axes('Parent',h);
    t=linspace(0,(duration-1)/10,duration);
    w=zeros(1,duration)*nan;
    w(1,700)=0;
    p=plot(t,w);
    x=45;
    axis([x x+50 -0.5 3]);
%     grid on;
    set(gcf,'Position',[50,160,540,90])
    get(gcf,'Position');
    ypos=1.5;
    g=text(x+12,ypos,['请减速' 32 '=' 32 num2str(0) 'km/h']);
    set(g,'String','NaN','Position',[x+6 ypos 0],'FontSize',20 )

    
    ax.YAxis.Visible='off';
    ax.XAxis.Visible='off';
    drawnow
    
end
%画图
Draw=0;
if Draw==1
    t=linspace(0,(duration-1)/10,duration);
    w=zeros(10,duration)*nan;
%     w(1,700)=0;
    p=plot(t,w,'.-');
    x=45;
    axis([x x+50 -0.2 0.5]);
    grid on;
    label=legend('T1','T2','T3','T4','T5','T6','T7','T8','T9','T10');
    set(gcf,'Position',[1,40,540,290])
    get(gcf,'Position');
    drawnow
    
end
% function add(polygonID, shape, color, fill, polygonType, layer, lineWidth)
traci.polygon.add('polygonID', {[-100 55] [-100 65]}, [255 255 255 255], false,'1',4,0.05);
traci.polygon.add('polygonID1', {[-100 86] [-100 89]}, [255 255 255 255], false,'1',4,0.05);
traci.polygon.add('polygonID2', {[112.8 -170] [116 -170]}, [255 0 0 255], false,'1',4,0.4);
traci.polygon.add('polygonID3', {[112.8 -210] [116 -210]}, [255 255 255 255], false,'1',4,0.05);
for traj_index=1:80
    traci.polygon.add(['traj' num2str(traj_index)], {[0 0] [0+0.01 0+0.01]}, [255 0 0 255], false,'1',4,0.05);
end
if usecase==137||usecase==140||usecase==141
    traci.polygon.add('polygonID4', {[112.8 -210] [116 -210]}, [255 0 0 255], false,'1',4,0.4);
end
for i = 1: duration
    traci.simulation.step();
    if i>700
          usecase
          t1=clock;
        if GlobVars.AEBDecision.AEBActive>0
            delay=delay+1;
        elseif exist('d_veh2goal','var')==1&&d_veh2goal<0.11
            delay=delay+1;
            if usecase==137||usecase==140||usecase==141
               delay=delay-0.5;
            end
        end
        if delay>=50
            traci.close();
            break
        end
        gIDList =traci.vehicle.getIDList();
        if isempty(gIDList)
            traci.close();
            break
        end
        if ~strcmp(gIDList,'S0')
            traci.close();
            break
        end
        %% 入参
        GoalLaneIndex=int16(1);
        d_veh2goal=200;
        TurnAroundActive=int16(0);
        LaneChangeActive=int16(0);
        PedestrianActive=int16(0);
        TrafficLightActive=int16(0);
        VehicleCrossingActive=int16(0);
        VehicleOncomingActive=int16(0);
        WidthOfLanes=[0,0,0,0,0,0];
        CurrentLaneFrontDis=200;
        CurrentLaneFrontVel=20;
        CurrentLaneFrontLen=5;
        LeftLaneBehindDis=-200;
        LeftLaneBehindVel=20;
        LeftLaneFrontDis=200;
        LeftLaneFrontVel=20;
        RightLaneBehindDis=-200;
        RightLaneBehindVel=-20;
        RightLaneFrontDis=200;
        RightLaneFrontVel=20;
        LeftLaneBehindLen=5;
        LeftLaneFrontLen=5;
        RightLaneBehindLen=5;
        RightLaneFrontLen=5;
        CurrentLaneFrontDisAvoidVehicle=200;
        CurrentLaneFrontVelAvoidVehicle=20;
        CurrentLaneFrontLenAvoidVehicle=5;
        TargetLaneBehindDisAvoidVehicle=-200;
        TargetLaneBehindVelAvoidVehicle=20;
        TargetLaneBehindLenAvoidVehicle=5;
        TargetLaneFrontDisAvoidVehicle=200;
        TargetLaneFrontVelAvoidVehicle=20;
        TargetLaneFrontLenAvoidVehicle=5;
        TargetLaneBehindDisAvoidVehicle1=-200;
        TargetLaneBehindVelAvoidVehicle1=20;
        TargetLaneBehindLenAvoidVehicle1=5;
        TargetLaneFrontDisAvoidVehicle1=200;
        TargetLaneFrontVelAvoidVehicle1=20;
        TargetLaneFrontLenAvoidVehicle1=5;
        TargetLaneBehindDisAvoidVehicle2=-200;
        TargetLaneBehindVelAvoidVehicle2=20;
        TargetLaneBehindLenAvoidVehicle2=5;
        TargetLaneFrontDisAvoidVehicle2=200;
        TargetLaneFrontVelAvoidVehicle2=20;
        TargetLaneFrontLenAvoidVehicle2=5;
        AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle=zeros(1,4,'double')-200;
        AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle=zeros(1,4,'double')+20;
        AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle=zeros(1,4,'double')+200;
        AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle=zeros(1,4,'double')+20;
        AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle=zeros(1,4,'double')+5;
        AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle=zeros(1,4,'double')+5;
        d_veh2int=200;
        d_veh2converge=200;
        d_veh2stopline=200;
%         s_ped=100;
%         v_ped=0;
        s_ped=zeros(1,40,'double');
        l_ped=zeros(1,40,'double');
        v_ped=zeros(1,40,'double')-1;
        psi_ped=zeros(1,40,'double');
        d_veh2cross=200;
        greenLight=1;
        time2nextSwitch=100;
        LanesWithFail=zeros(1,6,'int16');
        w_cross=6.4;
        d_veh2waitingArea=200;
        s_veh1=zeros(1,6,'double')+200;
        v_veh1=zeros(1,6,'double');
        d_veh2cross1=zeros(1,6,'double');
        s_veh1apostrophe1=zeros(1,6,'double')-200;
        s_veh_length=zeros(1,6,'double')+5;
        s_veh_width=zeros(1,6,'double')+1.8;
        s_vehapostrophe_length=zeros(1,6,'double')+5;
        s_vehapostrophe_width=zeros(1,6,'double')+1.8;
        IndexOfLaneOppositeCar=zeros([20,1],'int16');
        SpeedOppositeCar=zeros([20,1],'double');
        PosSOppositeCar=zeros([20,1],'double');
        PosSOppositeCar=PosSOppositeCar-200;
        LengthOppositeCar=zeros([20,1],'double')+5;
        IndexOfLaneCodirectCar=zeros([10,1],'int16');
        SpeedCodirectCar=zeros([10,1],'double')-1;
        PosSCodirectCar=zeros([10,1],'double');  
        LengthCodirectCar=zeros([20,1],'double')+5;
        traci.vehicle.setSpeedMode('S0', 0)
        % set view of sumo gui
        traci.gui.trackVehicle('View #0','S0');
        if usecase==101||usecase==102||usecase==103||usecase<=31||(usecase>=168&&usecase<=175)
            traci.gui.setZoom('View #0', 600);
        elseif usecase==110||usecase==111
            traci.gui.setZoom('View #0', 1800);
        elseif usecase==112||usecase==113||usecase==114
            traci.gui.setZoom('View #0', 2000);
        elseif usecase>=126&&usecase<=131
            traci.gui.setZoom('View #0', 2000);
        elseif usecase==147
            traci.gui.setZoom('View #0', 1500);
        elseif usecase>=132&&usecase<=144
            traci.gui.setZoom('View #0', 1500);
        elseif usecase==233
            traci.gui.setZoom('View #0', 1200);
        elseif usecase>=232&&usecase<=235
            traci.gui.setZoom('View #0', 1500);
        else
            traci.gui.setZoom('View #0', 1200);
        end
        
        %traci.gui.setBoundary('View #0',135,85,160,110);
%         yaw=traci.vehicle.getAngle('S0');
        lane=traci.vehicle.getLaneIndex('S0') ;
        speed=max([0 traci.vehicle.getSpeed('S0')]) ;
        current_road_ID=traci.vehicle.getRoadID('S0');
        current_lane_ID=traci.vehicle.getLaneID('S0');
        postion_veh=traci.vehicle.getPosition('S0');
        Parameter=traci.vehicle.getParameter('S0');
        Universal=traci.vehicle.getUniversal('S0');
        if i>=720
             route=global_route; 
        else 
        route = traci.vehicle.getRoute('S0');
        end
        if isempty(global_route)
           global_route= route;     
        elseif length(global_route)~=length(route)
            if any(strcmp(current_road_ID,global_route))
                global_route=global_route(1,find(strcmp(current_road_ID,global_route)):end);
            traci.vehicle.setRoute('S0',global_route);
            end
        end
%         RouteIndex=traci.vehicle.getRouteIndex('S0');
%         RouteID=traci.vehicle.getRouteID('S0');
%         routeIDlist=traci.route.getIDList;
%         RoutingMode=traci.vehicle.getRoutingMode('S0');
        NumOfLanes=traci.edge.getLaneNumber(current_road_ID);
      %% 当前车道与目标车道
      t5=clock;
      for WidthIndex=1:NumOfLanes 
          WidthOfLanes(WidthIndex)=3.2;
      end
        currentLanePosition=traci.vehicle.getLanePosition('S0');
        NumofLane=traci.edge.getLaneNumber(current_road_ID);
        CurrentLaneIndex=NumofLane-lane;
        if CurrentLaneIndex>6
            CurrentLaneIndex=1;
        end
        if strcmp(current_road_ID,'E15')&&currentLanePosition>25
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'E12')
            if usecase<=85
                TargetLaneIndex=2;
            elseif usecase==86||usecase==87||usecase==88
                TargetLaneIndex=3;
            elseif strcmp(route(find(strcmp(route,'E12'))+1),'E24')
                TargetLaneIndex=3;
            elseif strcmp(route(find(strcmp(route,'E12'))+1),'E22')
                TargetLaneIndex=2;
            elseif strcmp(route(find(strcmp(route,'E12'))+1),'E23')
                TargetLaneIndex=1;
            end
        elseif strcmp(current_road_ID,'E10')&&currentLanePosition>10
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'E25')&&(usecase==93||usecase==94)
            TargetLaneIndex=4;
        elseif strcmp(current_road_ID,'E25')&&(usecase==95||usecase==96||usecase==97)
            TargetLaneIndex=5;
        elseif strcmp(current_road_ID,'A0')&&usecase~=202&&usecase~=217
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'A1')&&usecase~=202&&usecase~=217
            TargetLaneIndex=3;
        elseif strcmp(current_road_ID,'A2')&&usecase~=202&&usecase~=217
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'A3')&&usecase~=202&&usecase~=217
            TargetLaneIndex=3;
        elseif strcmp(current_road_ID,'A4')&&usecase~=202&&usecase~=217
            TargetLaneIndex=4;
        elseif strcmp(current_road_ID,'A0')&&usecase==202
            TargetLaneIndex=1;
        elseif strcmp(current_road_ID,'A1')&&usecase==202
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'A2')&&usecase==202
            TargetLaneIndex=3;
        elseif strcmp(current_road_ID,'A3')&&usecase==202
            TargetLaneIndex=4;
        elseif strcmp(current_road_ID,'A4')&&usecase==202
            TargetLaneIndex=3;
        elseif strcmp(current_road_ID,'A0')&&usecase==217
            TargetLaneIndex=2;
            v_max=50/3.6;
        elseif strcmp(current_road_ID,'A1')&&usecase==217
            TargetLaneIndex=2;
            v_max=40/3.6;
        elseif strcmp(current_road_ID,'A2')&&usecase==217
            TargetLaneIndex=2;
            v_max=50/3.6;
        elseif strcmp(current_road_ID,'A3')&&usecase==217
            TargetLaneIndex=2;
            v_max=30/3.6;
        elseif strcmp(current_road_ID,'A4')&&usecase==217
            TargetLaneIndex=2;
            v_max=50/3.6;
        else
            TargetLaneIndex=CurrentLaneIndex;
        end
        if strcmp(current_road_ID,'8')&&currentLanePosition>100
            TargetLaneIndex=2;
        end
        if RefLaneIndex==0
            RefLaneIndex=CurrentLaneIndex;
        elseif (GlobVars.TrajPlanLaneChange.DurationLaneChange==0 || GlobVars.TrajPlanLaneChange.DurationLaneChange>GlobVars.TrajPlanLaneChange. t_lc_traj/0.1) ...
                && (GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan==0 || GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan> GlobVars.TrajPlanLaneChange_RePlan.t_lc_RePlan/0.1)
            RefLaneIndex=CurrentLaneIndex;
        end

        %% 本车frenet坐标
        if strcmp(current_road_ID,'E25')
            pos_l_CurrentLane=0;
            pos_s=postion_veh(2);
            if RefLaneIndex==1
                laneshape=traci.lane.getShape('E25_5');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==2
                laneshape=traci.lane.getShape('E25_4');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==3
                laneshape=traci.lane.getShape('E25_3');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);    
            elseif RefLaneIndex==4
                laneshape=traci.lane.getShape('E25_2');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);   
            elseif RefLaneIndex==5
                laneshape=traci.lane.getShape('E25_1');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);  
            elseif RefLaneIndex==6
                laneshape=traci.lane.getShape('E25_0');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            end
            if usecase>=132&&usecase<=144
                GoalLaneIndex=int16(6);
                d_veh2goal=-170-postion_veh(2);
                if usecase==137||usecase==140||usecase==141
                    d_veh2goal=d_veh2goal-40;
                end
            end
        elseif strcmp(current_road_ID,'E12')
            pos_l_CurrentLane=0;
            pos_s=postion_veh(2);
            if RefLaneIndex==1
                laneshape=traci.lane.getShape('E12_2');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==2
                laneshape=traci.lane.getShape('E12_1');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==3
                laneshape=traci.lane.getShape('E12_0');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            end
        elseif strcmp(current_road_ID,'E10')
            pos_l_CurrentLane=0;
            pos_s=postion_veh(2);
            if RefLaneIndex==1
                laneshape=traci.lane.getShape('E10_3');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==2
                laneshape=traci.lane.getShape('E10_2');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==3
                laneshape=traci.lane.getShape('E10_1');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==4
                laneshape=traci.lane.getShape('E10_0');
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            end           
        elseif strcmp(current_road_ID,'E15')
            pos_s=postion_veh(2);
            pos_l=-173.6-postion_veh(1);
            if pos_l>3.2
                pos_l_CurrentLane=4.8;
            elseif pos_l>0
                pos_l_CurrentLane=1.6;
            end
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
            pos_l_CurrentLane=0;
            pos_s=postion_veh(1);
            if RefLaneIndex==1
                laneshape=traci.lane.getShape([current_road_ID '_3']);
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==2
                laneshape=traci.lane.getShape([current_road_ID '_2']);
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);
            elseif RefLaneIndex==3
                laneshape=traci.lane.getShape([current_road_ID '_1']);
                pos_l=laneshape{1,1}(1,1)-postion_veh(1);    
            elseif RefLaneIndex==4
                laneshape=traci.lane.getShape([current_road_ID '_0']);
                pos_l=postion_veh(2)-laneshape{1,1}(1,2);   
            end
            
        elseif strcmp(current_road_ID,'8')
            pos_l_CurrentLane=0;
            pos_s=postion_veh(1);
            if RefLaneIndex==1
                laneshape=traci.lane.getShape([current_road_ID '_1']);
                pos_l=postion_veh(2)-laneshape{1,1}(1,2); 
            elseif RefLaneIndex==2
                laneshape=traci.lane.getShape([current_road_ID '_0']);
                pos_l=postion_veh(2)-laneshape{1,1}(1,2);
            end
        else
           pos_s=postion_veh(2);
           pos_l=101.6-postion_veh(1);
           pos_l_CurrentLane=0;
           
        end
        t6=clock;
         %% 搜寻前车
         if (usecase~=71&&usecase~=224&&usecase~=226&&usecase~=227&&usecase~=228&&usecase~=234&&usecase~=214&&usecase~=155)||(usecase==206&&strcmp(current_road_ID,'8')==0)
             if (usecase>=168&&usecase<=175)==0
             [aID, adist]=traci.vehicle.getLeader('S0',100);
             if isempty(aID)==0
                 CurrentLaneFrontLen=traci.vehicle.getLength(aID);
                 CurrentLaneFrontVel=traci.vehicle.getSpeed(aID);
%                  CurrentLaneFrontDis=adist+traci.vehicle.getMinGap('S0')+Parameters.l_veh;
                 CurrentLaneFrontDis=adist+traci.vehicle.getMinGap('S0')+CurrentLaneFrontLen;
                 if strcmp(traci.vehicle.getRoadID(aID),'8') || strcmp(traci.vehicle.getRoadID(aID),':9_2')
                     % 匝道汇入前
                     CurrentLaneFrontVelAvoidVehicle=traci.vehicle.getSpeed(aID);
                     CurrentLaneFrontDisAvoidVehicle=adist+traci.vehicle.getMinGap('S0')+Parameters.l_veh;
                 else
                     CurrentLaneFrontVelAvoidVehicle=traci.vehicle.getSpeed(aID);
                     CurrentLaneFrontDisAvoidVehicle=adist+traci.vehicle.getMinGap('S0')+Parameters.l_veh;
                 end
             end
             end
         end
         t7=clock;
   %% 目标车道
        if strcmp(current_road_ID,'E25')&&strcmp(route{length(route)-1},'E25')
            if CurrentLaneFrontDis<=25&&usecase==42
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif CurrentLaneFrontDis<=20&&usecase==43
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif CurrentLaneFrontDis<=15&&usecase==44
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif usecase==45|| usecase==46||usecase==61||usecase==62||usecase==63
                TargetLaneIndex=4;
            elseif usecase>=132&&usecase<=143
                TargetLaneIndex=6;
            elseif usecase==144
                TargetLaneIndex=5;
            end         
        end
        %% 确定掉头对向道路信息
        
        if strcmp(current_road_ID,'7')
            LaneNumber=traci.edge.getLaneNumber('5');
            oppositeLane1Width=traci.lane.getWidth('5_2');
            oppositeLane2Width=traci.lane.getWidth('5_1');
            oppositeLane3Width=traci.lane.getWidth('5_0');
            CurrentLaneWidth=traci.lane.getWidth('7_1');
            NumOfLanesOpposite=LaneNumber;
            WidthOfLanesOpposite=[oppositeLane1Width,oppositeLane2Width,oppositeLane3Width,0,0,0];
            WidthOfGap=0;
%             WidthOfLaneCurrent=CurrentLaneWidth;
            WidthOfLanes=[traci.lane.getWidth('7_1'),traci.lane.getWidth('7_0'),0,0,0,0];
%             CurrentLaneIndex=traci.edge.getLaneNumber('7')-traci.vehicle.getLaneIndex('S0');
            s_turnaround_border=-6.4;
        elseif strcmp(current_road_ID,'10')
            if usecase==15||usecase==16
                NumOfLanesOpposite=1;
            else
                NumOfLanesOpposite=traci.edge.getLaneNumber('9');
            end
            WidthOfLanesOpposite=[traci.lane.getWidth('9_1'),traci.lane.getWidth('9_0'),0,0,0,0];
            WidthOfGap=0;
%             WidthOfLaneCurrent=traci.lane.getWidth('10_1');
            WidthOfLanes=[traci.lane.getWidth('10_1'),traci.lane.getWidth('10_0'),0,0,0,0];
%             CurrentLaneIndex=traci.edge.getLaneNumber('10')-traci.vehicle.getLaneIndex('S0');
            s_turnaround_border=146.8;
        elseif strcmp(current_road_ID,'gneE0')
            LaneNumber=traci.edge.getLaneNumber('-gneE0');
            if LaneNumber==2
                NumOfLanesOpposite=1;
                WidthOfLanesOpposite=[3.2,0,0,0,0,0];
                WidthOfGap=3.2;
                WidthOfLanes=[traci.lane.getWidth('gneE0_0'),0,0,0,0,0];
%                 CurrentLaneIndex=traci.edge.getLaneNumber('gneE0')-traci.vehicle.getLaneIndex('S0');
                s_turnaround_border=246.8;
            else
                NumOfLanesOpposite=1;
                WidthOfLanesOpposite=[3.2,0,0,0,0,0];
                WidthOfGap=0;
                WidthOfLanes=[traci.lane.getWidth('gneE0_0'),0,0,0,0,0];
%                 CurrentLaneIndex=traci.edge.getLaneNumber('gneE0')-traci.vehicle.getLaneIndex('S0');
                s_turnaround_border=246.8;
            end
        elseif strcmp(current_road_ID,'E25')&&length(route)>4&&strcmp(route(find(strcmp(route,'E25'))+3),'7')
            NumOfLanesOpposite=0;
            WidthOfLanesOpposite=[3.2,3.2,3.2,0,0,0];
            WidthOfGap=0;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=0;
            WidthOfLanes=[3.2,3.2,3.2,3.2,3.2,3.2];
%             CurrentLaneIndex=6-lane;
            TargetLaneIndex=2;
        else
            NumOfLanesOpposite=0;
            WidthOfLanesOpposite=[3.2,3.2,3.2,0,0,0];
            WidthOfGap=0;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=0;
        end    
        t8=clock;
        %% 确定与路口的距离和停止线距离
        if strcmp(current_road_ID,'7')%第一个路口
            d_veh2int=-10.4-pos_s;
        elseif usecase>31&&(strcmp(current_road_ID,':8_9')||strcmp(current_road_ID,':8_11')||strcmp(current_road_ID,':8_19'))%第一个路口
            d_veh2int=-10.4-pos_s;
        elseif strcmp(current_road_ID,'10')
            d_veh2int=142.8-pos_s;
        elseif strcmp(current_road_ID,'gneE0')
            d_veh2int=242.8-pos_s;
        elseif strcmp(current_road_ID,'9')
            d_veh2int=postion_veh(2)-10.4;
        elseif strcmp(current_road_ID,'8')
            d_veh2int=postion_veh(1)-(-92.80);
        elseif strcmp(current_road_ID,'E13')
            d_veh2int=77.24-currentLanePosition;
        elseif strcmp(current_road_ID,'E5')%十字路口左转
            d_veh2int=89.6-currentLanePosition;
        elseif strcmp(current_road_ID,'E15')
            d_veh2int=97.54-currentLanePosition;
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
            d_veh2int=1000-pos_s;
        end
        if d_veh2int<0&&~strcmp(current_road_ID,':8_9')%第一个路口
            d_veh2int=200;
        end
        if strcmp(current_road_ID,'E12')
            d_veh2int=traci.lane.getLength('E12_0')-currentLanePosition;
        end
        d_veh2trafficStopline=d_veh2int;
        %% 确定与汇入点距离
        if strcmp(current_road_ID,'9')
            d_veh2converge=max([132.40+11.73-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5.5-4;
        elseif strcmp(current_road_ID,':8_0')
            d_veh2converge=max([11.73-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5.5-4;
        elseif strcmp(current_road_ID,'E2')
            d_veh2converge=max([84.85+9.42-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-6-4;
        elseif strcmp(current_road_ID,':J3_0')
            d_veh2converge=max([9.42-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-6-4;
        elseif strcmp(current_road_ID,'E3')%车道汇聚
            d_veh2converge=max([86.08+3-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-12-4;
        elseif strcmp(current_road_ID,':J4_1')%车道汇聚
            d_veh2converge=max([3-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-12-4;
        elseif strcmp(current_road_ID,'E5')%J6路口右转
            d_veh2converge=max([89.6+6.56-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-1-4;
        elseif strcmp(current_road_ID,':J6_7')%J6路口右转
            d_veh2converge=max([6.56-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-1-4;
        elseif strcmp(current_road_ID,'8')%丁字路口9右转
            d_veh2converge=max([179.2+9.03-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5-4-0.5;
        elseif strcmp(current_road_ID,':9_2')%丁字路口9右转
            d_veh2converge=max([9.03-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5-4; 
        elseif usecase==101&&strcmp(current_road_ID,'2')&&strcmp(route(find(strcmp(route,'2'))+1),'3')%环岛
            d_veh2converge=max([traci.lane.getLength('2_0')+traci.lane.getLength(':gneJ11_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ11_0_0')+0;
        elseif usecase==101&&strcmp(current_road_ID,':gneJ11_0')%环岛
            d_veh2converge=max([traci.lane.getLength(':gneJ11_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ11_0_0')+0;
        elseif (usecase==102||usecase==103)&&strcmp(current_road_ID,'2')&&strcmp(route(find(strcmp(route,'2'))+1),'a8')%环岛
            d_veh2converge=max([traci.lane.getLength('2_0')+5.53+4.5-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5-4;
        elseif (usecase==102||usecase==103)&&strcmp(current_road_ID,':gneJ11_1')%环岛
            d_veh2converge=max([5.53+4.5-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5-4;
        elseif (usecase==102||usecase==103)&&strcmp(current_road_ID,':gneJ11_4')%环岛
            d_veh2converge=max([4.5-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-5-4;
        elseif usecase==102&&strcmp(current_road_ID,'a8')&&strcmp(route(find(strcmp(route,'a8'))+1),'13')%环岛
            d_veh2converge=max([traci.lane.getLength('a8_0')+traci.lane.getLength(':gneJ13_2_0')/2-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ13_2_0')/2+0;
        elseif usecase==102&&strcmp(current_road_ID,':gneJ13_2')%环岛
            d_veh2converge=max([traci.lane.getLength(':gneJ13_2_0')/2-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ13_2_0')/2+0;
        elseif usecase==103&&strcmp(current_road_ID,'a8')&&strcmp(route(find(strcmp(route,'a8'))+1),'12')%环岛
            d_veh2converge=max([traci.lane.getLength('a8_0')+traci.lane.getLength(':gneJ13_3_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ13_3_0')+0;
        elseif usecase==103&&strcmp(current_road_ID,':gneJ13_3')%环岛
            d_veh2converge=max([traci.lane.getLength(':gneJ13_3_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ13_3_0')+0; 
        elseif usecase==103&&strcmp(current_road_ID,'12')&&strcmp(route(find(strcmp(route,'12'))+1),'6')%环岛
            d_veh2converge=max([traci.lane.getLength('12_0')+traci.lane.getLength(':gneJ14_2_0')/2-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ14_2_0')/2+0;
        elseif usecase==103&&strcmp(current_road_ID,':gneJ14_2')%环岛
            d_veh2converge=max([traci.lane.getLength(':gneJ14_2_0')/2-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':gneJ14_2_0')/2+0; 
        elseif strcmp(current_road_ID,'E40')%三车道汇聚
            d_veh2converge=max([traci.lane.getLength('E40_0')+traci.lane.getLength(':J37_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':J37_0_0');
        elseif strcmp(current_road_ID,':J37_0')%三车道汇聚
            d_veh2converge=max([traci.lane.getLength(':J37_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':J37_0_0');
        elseif strcmp(current_road_ID,'E45')%四车道汇聚
            d_veh2converge=max([traci.lane.getLength('E40_0')+traci.lane.getLength(':J43_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':J43_0_0')-0.5;
        elseif strcmp(current_road_ID,':J43_0')%四车道汇聚
            d_veh2converge=max([traci.lane.getLength(':J43_0_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':J43_0_0')-0.5;
        elseif strcmp(current_road_ID,'-E34')&&usecase>=150&&usecase<=151%无信灯路口右转有停车让行标志
            d_veh2converge=max([traci.lane.getLength('-E34_0')+traci.lane.getLength(':J30_6_0')-currentLanePosition 0]);
            d_veh2stopline=d_veh2converge-traci.lane.getLength(':J30_6_0'); 
        end

        %% 搜寻匝道汇入时主路上车辆
        if strcmp(current_road_ID,'9') || strcmp(current_road_ID,':8_0')%十字路口右转
%             vehicles_targetLaneIDs=[traci.edge.getLastStepVehicleIDs('13') traci.edge.getLastStepVehicleIDs(':8_5') traci.edge.getLastStepVehicleIDs('8')];
%             [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,86.4,d_veh2converge,'X','-');
%             TargetLaneBehindDisAvoidVehicle=s_r;
%             TargetLaneBehindVelAvoidVehicle=v_r;
%             TargetLaneFrontDisAvoidVehicle=s_f;
%             TargetLaneFrontVelAvoidVehicle=v_f;
            
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('13_0',':8_5_0','8_0',traci.lane.getLength('13_0'),traci.lane.getLength(':8_5_0'),traci.lane.getLength(':8_5_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('9_0',':8_0_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif strcmp(current_road_ID,'E2') || strcmp(current_road_ID,':J3_0')%匝道汇入
%             vehicles_targetLaneIDs=[traci.edge.getLastStepVehicleIDs('E0') traci.edge.getLastStepVehicleIDs(':J3_1') traci.edge.getLastStepVehicleIDs('E3')];
%             [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,162.3,d_veh2converge,'Y','+');
%             TargetLaneBehindDisAvoidVehicle=s_r;
%             TargetLaneBehindVelAvoidVehicle=v_r;
%             TargetLaneFrontDisAvoidVehicle=s_f;
%             TargetLaneFrontVelAvoidVehicle=v_f;

            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E0_0',':J3_1_0','E3_0',traci.lane.getLength('E0_0'),traci.lane.getLength(':J3_1_0'),traci.lane.getLength(':J3_1_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('E2_0',':J3_0_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif strcmp(current_road_ID,'E3') || strcmp(current_road_ID,':J4_1')%车道汇聚
%             vehicles_targetLaneIDs=[traci.edge.getLastStepVehicleIDs('-E9') traci.edge.getLastStepVehicleIDs(':J4_2')];
%             [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,251.2,d_veh2converge,'Y','+');
%             TargetLaneBehindDisAvoidVehicle=s_r;
%             TargetLaneBehindVelAvoidVehicle=v_r;
%             TargetLaneFrontDisAvoidVehicle=s_f;
%             TargetLaneFrontVelAvoidVehicle=v_f;  

            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('-E9_0',':J4_2_0','E13_0',traci.lane.getLength('-E9_0'),traci.lane.getLength(':J4_2_0'),traci.lane.getLength(':J4_2_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('E3_0',':J4_1_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif strcmp(current_road_ID,'E5') || strcmp(current_road_ID,':J6_7')%十字路口右转
%             vehicles_targetLaneIDs=[traci.lane.getLastStepVehicleIDs('-E8_0') traci.lane.getLastStepVehicleIDs(':J6_11_0') traci.lane.getLastStepVehicleIDs('E7_0')];
%             [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,-292.8,d_veh2converge,'X','+');
%             TargetLaneBehindDisAvoidVehicle=s_r;
%             TargetLaneBehindVelAvoidVehicle=v_r;
%             TargetLaneFrontDisAvoidVehicle=s_f;
%             TargetLaneFrontVelAvoidVehicle=v_f;

            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('-E8_0',':J6_11_0','E7_0',traci.lane.getLength('-E8_0'),traci.lane.getLength(':J6_11_0'),traci.lane.getLength(':J6_11_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('E5_0',':J6_7_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif strcmp(current_road_ID,'8') || strcmp(current_road_ID,':9_2')%丁字路口右转
%             vehicles_targetLaneIDs=[traci.lane.getLastStepVehicleIDs('-E20_0') traci.lane.getLastStepVehicleIDs(':9_5_0') traci.lane.getLastStepVehicleIDs('E1_0')];
%             [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,10.2,d_veh2converge,'Y','+');
%             TargetLaneBehindDisAvoidVehicle=s_r;
%             TargetLaneBehindVelAvoidVehicle=v_r;
%             TargetLaneFrontDisAvoidVehicle=s_f;
%             TargetLaneFrontVelAvoidVehicle=v_f;  

            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('-E20_0',':9_5_0','E1_0',traci.lane.getLength('-E20_0'),traci.lane.getLength(':9_5_0'),traci.lane.getLength(':9_5_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('8_0',':9_2_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif usecase==101&&(strcmp(current_road_ID,'2') || strcmp(current_road_ID,':gneJ11_0'))&&strcmp(route(find(strcmp(route,'2'))+1),'3')%%环岛
            [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform2('7_0',':gneJ11_2_0','3_0',traci.lane.getLength('7_0'),traci.lane.getLength(':gneJ11_2_0'),17.93,d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
        elseif (usecase==102||usecase==103)&&(strcmp(current_road_ID,'2') || strcmp(current_road_ID,':gneJ11_1')|| strcmp(current_road_ID,':gneJ11_4'))&&strcmp(route(find(strcmp(route,'2'))+1),'a8')%%环岛
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('7_0') traci.lane.getLastStepVehicleIDs(':gneJ11_2_0') traci.lane.getLastStepVehicleIDs(':gneJ11_3_0') traci.lane.getLastStepVehicleIDs('a8_0') ];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id});
                oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
                if strcmp(oncomingVehicleRoadLaneID,'7_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(traci.lane.getLength('7_0')+10-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ11_2_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(10-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ11_3_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(10-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,'a8_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(-(traci.lane.getLength('a8_0')-10)-oncomingVehiclePosition);
                end
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>0);
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<=0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>0)));
                    s_f=oncomingVehiclesPositions(veh1ID);
                    v_f=oncomingVehiclesSpeeds(veh1ID);
                else
                    s_f=200;
                    v_f=20;
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
                    s_r=oncomingVehiclesPositions(veh1apostrophe1ID);
                    v_r=oncomingVehiclesSpeeds(veh1apostrophe1ID);
                else
                    s_r=-200;
                    v_r=20;
                end
            else
                s_f=200;
                v_f=20;
                s_r=-200;
            end
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
        elseif usecase==102&&(strcmp(current_road_ID,'a8') || strcmp(current_road_ID,':gneJ13_2'))&&strcmp(route(find(strcmp(route,'a8'))+1),'13')%%环岛
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('9_0') traci.lane.getLastStepVehicleIDs(':gneJ13_0_0') traci.lane.getLastStepVehicleIDs(':gneJ13_1_0') traci.lane.getLastStepVehicleIDs(':gneJ13_4_0') traci.lane.getLastStepVehicleIDs('13_0') ];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id});
                oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
                if strcmp(oncomingVehicleRoadLaneID,'9_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(traci.lane.getLength('9_0')+10-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ13_0_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(9-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ13_1_0')||strcmp(oncomingVehicleRoadLaneID,':gneJ13_4_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(9-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,'13_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(-(traci.lane.getLength('13_0')-10)-oncomingVehiclePosition);
                end
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>=0);
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>=0)));
                    s_f=oncomingVehiclesPositions(veh1ID);
                    v_f=oncomingVehiclesSpeeds(veh1ID);
                else
                    s_f=200;
                    v_f=20;
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
                    s_r=oncomingVehiclesPositions(veh1apostrophe1ID);
                    v_r=oncomingVehiclesSpeeds(veh1apostrophe1ID);
                else
                    s_r=-200;
                    v_r=20;
                end
            else
                s_f=200;
                v_f=20;
                s_r=-200;
            end
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
        elseif usecase==103&&(strcmp(current_road_ID,'a8') || strcmp(current_road_ID,':gneJ13_3'))&&strcmp(route(find(strcmp(route,'a8'))+1),'12')%%环岛
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('9_0') traci.lane.getLastStepVehicleIDs(':gneJ13_1_0') traci.lane.getLastStepVehicleIDs(':gneJ13_4_0') traci.lane.getLastStepVehicleIDs('12_0') ];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id});
                oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
                if strcmp(oncomingVehicleRoadLaneID,'9_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(traci.lane.getLength('9_0')+18-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ13_1_0')||strcmp(oncomingVehicleRoadLaneID,':gneJ13_4_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(18-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,'12_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(-(traci.lane.getLength('12_0')-18)-oncomingVehiclePosition);
                end
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>=0);
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>=0)));
                    s_f=oncomingVehiclesPositions(veh1ID);
                    v_f=oncomingVehiclesSpeeds(veh1ID);
                else
                    s_f=200;
                    v_f=20;
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
                    s_r=oncomingVehiclesPositions(veh1apostrophe1ID);
                    v_r=oncomingVehiclesSpeeds(veh1apostrophe1ID);
                else
                    s_r=-200;
                    v_r=20;
                end
            else
                s_f=200;
                v_f=20;
                s_r=-200;
            end
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
        elseif usecase==103&&(strcmp(current_road_ID,'12') || strcmp(current_road_ID,':gneJ13_2'))&&strcmp(route(find(strcmp(route,'12'))+1),'6')%%环岛
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('11_0') traci.lane.getLastStepVehicleIDs(':gneJ13_0_0') traci.lane.getLastStepVehicleIDs(':gneJ14_1_0')  traci.lane.getLastStepVehicleIDs('6_0') ];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id});
                oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
                if strcmp(oncomingVehicleRoadLaneID,'11_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(traci.lane.getLength('11_0')+10-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ14_0_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(9-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,':gneJ14_1_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(9-oncomingVehiclePosition);
                elseif strcmp(oncomingVehicleRoadLaneID,'6_0')
                    oncomingVehiclesPositions(id)=d_veh2converge-(-(traci.lane.getLength('6_0')-10)-oncomingVehiclePosition);
                end
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>=0);
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>=0)));
                    s_f=oncomingVehiclesPositions(veh1ID);
                    v_f=oncomingVehiclesSpeeds(veh1ID);
                else
                    s_f=200;
                    v_f=20;
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
                    s_r=oncomingVehiclesPositions(veh1apostrophe1ID);
                    v_r=oncomingVehiclesSpeeds(veh1apostrophe1ID);
                else
                    s_r=-200;
                    v_r=20;
                end
            else
                s_f=200;
                v_f=20;
                s_r=-200;
            end
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
        elseif (strcmp(current_road_ID,'E40') || strcmp(current_road_ID,':J37_0'))%三车道汇聚
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E37_0',':J37_2_0','E38_0',traci.lane.getLength('E37_0'),traci.lane.getLength(':J37_2_0'),traci.lane.getLength(':J37_2_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E39_0',':J37_1_0','E38_0',traci.lane.getLength('E39_0'),traci.lane.getLength(':J37_1_0'),traci.lane.getLength(':J37_1_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle1=s_r;
            TargetLaneBehindVelAvoidVehicle1=v_r;
            TargetLaneBehindLenAvoidVehicle1=l_r;
            TargetLaneFrontDisAvoidVehicle1=s_f;
            TargetLaneFrontVelAvoidVehicle1=v_f;
            TargetLaneFrontLenAvoidVehicle1=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('E40_0',':J37_0_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif (strcmp(current_road_ID,'E45') || strcmp(current_road_ID,':J43_0'))%四车道汇聚
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E41_0',':J43_3_0','E44_0',traci.lane.getLength('E41_0'),traci.lane.getLength(':J43_3_0'),traci.lane.getLength(':J43_3_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E42_0',':J43_2_0','E44_0',traci.lane.getLength('E42_0'),traci.lane.getLength(':J43_2_0'),traci.lane.getLength(':J43_2_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle1=s_r;
            TargetLaneBehindVelAvoidVehicle1=v_r;
            TargetLaneBehindLenAvoidVehicle1=l_r;
            TargetLaneFrontDisAvoidVehicle1=s_f;
            TargetLaneFrontVelAvoidVehicle1=v_f;
            TargetLaneFrontLenAvoidVehicle1=l_f;
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E43_0',':J43_1_0','E44_0',traci.lane.getLength('E43_0'),traci.lane.getLength(':J43_1_0'),traci.lane.getLength(':J43_1_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle2=s_r;
            TargetLaneBehindVelAvoidVehicle2=v_r;
            TargetLaneBehindLenAvoidVehicle2=l_r;
            TargetLaneFrontDisAvoidVehicle2=s_f;
            TargetLaneFrontVelAvoidVehicle2=v_f;
            TargetLaneFrontLenAvoidVehicle2=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('E45_0',':J43_0_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        elseif (strcmp(current_road_ID,'-E34') || strcmp(current_road_ID,':J30_6_0'))&&usecase>=150&&usecase<=151%无信号的路口右转有停车让行标志
            [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3('E32_0',':J30_10_0','E35_0',traci.lane.getLength('E32_0'),traci.lane.getLength(':J30_10_0'),traci.lane.getLength(':J30_10_0'),d_veh2converge);
            TargetLaneBehindDisAvoidVehicle=s_r;
            TargetLaneBehindVelAvoidVehicle=v_r;
            TargetLaneBehindLenAvoidVehicle=l_r;
            TargetLaneFrontDisAvoidVehicle=s_f;
            TargetLaneFrontVelAvoidVehicle=v_f;
            TargetLaneFrontLenAvoidVehicle=l_f;
            CurrentLaneFrontVel=v_a;
            CurrentLaneFrontDis=s_a;
            CurrentLaneFrontLen=l_a;
            [s_rampf,v_rampf,l_rampf]=RampVehInform3('-E34_0',':J30_6_0',d_veh2converge);
            CurrentLaneFrontDisAvoidVehicle=s_rampf;
            CurrentLaneFrontVelAvoidVehicle=v_rampf;
            CurrentLaneFrontLenAvoidVehicle=l_rampf;
        end 
       %% 确定与人行横道距离
        if strcmp(current_road_ID,'E1')||strcmp(current_road_ID,':J2_2')
            % d_veh2cross=max([80-currentLanePosition-widthCrossing 0]);
            d_veh2cross=75.3-postion_veh(2);
        elseif strcmp(current_road_ID,'E5')&&strcmp(route(find(strcmp(route,'E5'))+1),'E8')%J6路口左转
            d_veh2cross=max([8.97+7.89+89.6-2.5-currentLanePosition 0]);
        elseif strcmp(current_road_ID,':J6_9') %J6路口左转
            d_veh2cross=max([8.97+7.89-2.5-currentLanePosition 0]);
        elseif strcmp(current_road_ID,':J6_18')%J6路口左转
            d_veh2cross=max([8.97-2.5-currentLanePosition 0]);
        elseif strcmp(current_road_ID,'E5')&&strcmp(route(find(strcmp(route,'E5'))+1),'E7') %J6路口右转
            d_veh2cross=max([traci.lane.getLength(':J6_17_0')+6.56+89.6-2.5-currentLanePosition 0]);
        elseif strcmp(current_road_ID,':J6_7')%J6路口右转
            d_veh2cross=max([traci.lane.getLength(':J6_17_0')+6.56-2.5-currentLanePosition 0]); 
        end
        if usecase>=112
            if (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_7')||strcmp(current_road_ID,'E33'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'E33')%无红绿灯路口直行
                w_cross=2.5;
                if (strcmp(current_road_ID,':J30_7')&&currentLanePosition>w_cross)
                    d_veh2cross=traci.lane.getLength(':J30_7_0')-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,'E33')
                    d_veh2cross=-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,':J30_7')
                    d_veh2cross=-currentLanePosition;
                else
                    d_veh2cross=traci.lane.getLength('-E34_0')-currentLanePosition;
                end
            elseif (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_8')||strcmp(current_road_ID,'-E32'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'-E32')%无红绿灯路口左转
                w_cross=2.5;
                if (strcmp(current_road_ID,':J30_8')&&currentLanePosition>w_cross)
                    d_veh2cross=traci.lane.getLength(':J30_8_0')-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,'-E32')
                    d_veh2cross=-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,':J30_8')
                    d_veh2cross=-currentLanePosition;
                else
                    d_veh2cross=traci.lane.getLength('-E34_0')-currentLanePosition;
                end
            elseif (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_6')||strcmp(current_road_ID,'E35'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'E35')%无红绿灯路口右转
                w_cross=2.5;
                if (strcmp(current_road_ID,':J30_6')&&currentLanePosition>w_cross)
                    d_veh2cross=traci.lane.getLength(':J30_6_0')-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,'E35')
                    d_veh2cross=-currentLanePosition-w_cross;
                elseif strcmp(current_road_ID,':J30_6')
                    d_veh2cross=-currentLanePosition;
                else
                    d_veh2cross=traci.lane.getLength('-E34_0')-currentLanePosition;
                end
            end
        end
        if strcmp(current_road_ID,'E5')||strcmp(current_road_ID,':J6_9')||strcmp(current_road_ID,':J6_18')||strcmp(current_road_ID,':J6_7')
            w_cross=2.5;
        end
       %% 确定行人位置、速度
        if strcmp(current_road_ID,'E1')||strcmp(current_road_ID,':J2_2')||strcmp(current_road_ID,'E2')
            if (usecase>=126&&usecase<=130)||(usecase>=218&&usecase<=219)||usecase==232
                [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('E1_0',':J2_w0','E10','E12',pos_s);
            else
                [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('E1_0',':J2_w0',[],[],pos_s);
            end
        elseif (strcmp(current_road_ID,'E5')||strcmp(current_road_ID,':J6_9')||strcmp(current_road_ID,':J6_18'))&&strcmp(route(find(strcmp(route,'E5'))+1),'E8')%J6路口左转
            [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('E5_0',':J6_9_0',':J6_18_0',':J6_c1',pos_s,current_lane_ID,'turnright');%左转
         elseif (strcmp(current_road_ID,'E5')||strcmp(current_road_ID,':J6_7'))&&strcmp(route(find(strcmp(route,'E5'))+1),'E7')%J6路口右转
             [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('E5_0',':J6_7_0',':J6_17_0',':J6_c0',pos_s,current_lane_ID,'turnright');%右转
        elseif (strcmp(current_road_ID,'-E34')||(strcmp(current_road_ID,':J30_7')&&currentLanePosition<w_cross)||(strcmp(current_road_ID,':J30_8')...
                &&currentLanePosition<w_cross)||(strcmp(current_road_ID,':J30_6')&&currentLanePosition<w_cross))&&usecase>=112&&usecase<145%无红绿灯路口直行或左转或右转
             [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('-E34_0',':J30_c2',[],[],pos_s);
        elseif strcmp(current_road_ID,':J30_7')&&usecase>=112&&usecase<145%无红绿灯路口直行
            [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform(':J30_7_0',':J30_c0',[],[],pos_s);
        elseif strcmp(current_road_ID,':J30_8')&&usecase>=112&&usecase<145%无红绿灯路口左转
            [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('-E34_0',':J30_8_0',[],':J30_c3',pos_s,current_lane_ID,'turnleft');
        elseif strcmp(current_road_ID,':J30_6')&&usecase>=112&&usecase<145%无红绿灯路口右转
            [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('-E34_0',':J30_6_0',[],':J30_c1',pos_s,current_lane_ID,'turnright');%右转
        end
        %% 确定与待转区距离 
        if strcmp(current_road_ID,'E13')
            d_veh2cross1(1)=max([3.2+4.18+77.24-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-4 0]);
        elseif strcmp(current_road_ID,':J12_4') 
            d_veh2cross1(1)=max([3.2+4.18-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-4 0]);
        elseif strcmp(current_road_ID,':J12_7')
            d_veh2cross1(1)=max([3.2-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-4 0]);
        elseif strcmp(current_road_ID,'E5')
            d_veh2cross1(1)=max([5+7.89+89.6-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-5.5 0]); 
        elseif strcmp(current_road_ID,':J6_9') 
            d_veh2cross1(1)=max([5+7.89-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-5.5 0]);
        elseif strcmp(current_road_ID,':J6_18')
            d_veh2cross1(1)=max([5-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-5.5 0]);
        elseif strcmp(current_road_ID,'7')&&usecase~=170
            d_veh2cross1(1)=max([3+6.41+traci.lane.getLength('7_1')-currentLanePosition 0]);
            d_veh2cross1(2)=max([4+3+6.41+traci.lane.getLength('7_1')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-3 0]); 
        elseif strcmp(current_road_ID,':8_11') 
            d_veh2cross1(1)=max([3+6.41-currentLanePosition 0]);
            d_veh2cross1(2)=max([4+3+6.41-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-3 0]);
        elseif strcmp(current_road_ID,':8_19')
            d_veh2cross1(1)=max([3-currentLanePosition 0]);
            d_veh2cross1(2)=max([4+3-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-3 0]);
        elseif strcmp(current_road_ID,'-E34')&&strcmp(route(find(strcmp(route,'-E34'))+1),'E33')%无红绿灯路口直行
            d_veh2cross1(1)=max([6+traci.lane.getLength('-E34_0')-currentLanePosition 0]);    
            d_veh2cross1(2)=max([1.6+6+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(3)=max([1.6+6+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(4)=max([3.2+6+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(5)=max([traci.lane.getLength(':J30_7_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(6)=max([traci.lane.getLength(':J30_7_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-6 0]);            
        elseif strcmp(current_road_ID,':J30_7')
            d_veh2cross1(1)=max([6-currentLanePosition 0]);    
            d_veh2cross1(2)=max([1.6+6-currentLanePosition 0]);
            d_veh2cross1(3)=max([1.6+6-currentLanePosition 0]);
            d_veh2cross1(4)=max([3.2+6-currentLanePosition 0]);
            d_veh2cross1(5)=max([traci.lane.getLength(':J30_7_0')-currentLanePosition 0]);
            d_veh2cross1(6)=max([traci.lane.getLength(':J30_7_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-6 0]); 
        elseif strcmp(current_road_ID,'-E34')&&strcmp(route(find(strcmp(route,'-E34'))+1),'-E32')%无红绿灯路口左转
            d_veh2cross1(1)=max([traci.lane.getLength(':J30_8_0')/2-1+traci.lane.getLength('-E34_0')-currentLanePosition 0]);    
            d_veh2cross1(2)=max([traci.lane.getLength(':J30_8_0')/2-1+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(3)=max([traci.lane.getLength(':J30_8_0')/2+1+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(4)=max([traci.lane.getLength(':J30_8_0')/2+1+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(5)=max([traci.lane.getLength(':J30_8_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2cross1(6)=max([traci.lane.getLength(':J30_8_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-5.93-0.2 0]);
        elseif strcmp(current_road_ID,':J30_8')%无红绿灯路口左转
            d_veh2cross1(1)=max([traci.lane.getLength(':J30_8_0')/2-1-currentLanePosition 0]);    
            d_veh2cross1(2)=max([traci.lane.getLength(':J30_8_0')/2-1-currentLanePosition 0]);
            d_veh2cross1(3)=max([traci.lane.getLength(':J30_8_0')/2+1-currentLanePosition 0]);
            d_veh2cross1(4)=max([traci.lane.getLength(':J30_8_0')/2+1-currentLanePosition 0]);
            d_veh2cross1(5)=max([traci.lane.getLength(':J30_8_0')-currentLanePosition 0]);
            d_veh2cross1(6)=max([traci.lane.getLength(':J30_8_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-5.93-0.2 0]); 
        elseif strcmp(current_road_ID,'-E34')&&strcmp(route(find(strcmp(route,'-E34'))+1),'E35')%无红绿灯路口右转
            d_veh2cross1(1)=max([traci.lane.getLength(':J30_6_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);    
            d_veh2cross1(2)=max([traci.lane.getLength(':J30_6_0')+traci.lane.getLength('-E34_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J30_6_0')/3 0]);
        elseif strcmp(current_road_ID,':J30_6')%无红绿灯路口右转
            d_veh2cross1(1)=max([traci.lane.getLength(':J30_6_0')-currentLanePosition 0]);    
            d_veh2cross1(2)=max([traci.lane.getLength(':J30_6_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J30_6_0')/3 0]); 
        elseif strcmp(current_road_ID,'E36')&&strcmp(route(find(strcmp(route,'E36'))+1),'E38')%无红绿灯Y字路口左转
            d_veh2cross1(1)=max([(traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0'))/2+traci.lane.getLength('E36_0')-currentLanePosition 0]);    
            d_veh2cross1(2)=max([(traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0'))/2+traci.lane.getLength('E36_0')-currentLanePosition 0]);
            d_veh2cross1(3)=max([traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0')+traci.lane.getLength('E36_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J34_3_0') 0]);
        elseif strcmp(current_road_ID,':J34_3')%无红绿灯Y字路口左转
            d_veh2cross1(1)=max([(traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0'))/2-currentLanePosition 0]);    
            d_veh2cross1(2)=max([(traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0'))/2-currentLanePosition 0]);
            d_veh2cross1(3)=max([traci.lane.getLength(':J34_3_0')+traci.lane.getLength(':J34_6_0')-currentLanePosition 0]);
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J34_3_0') 0]);
        elseif strcmp(current_road_ID,'E36')&&strcmp(route(find(strcmp(route,'E36'))+1),'E37')%无红绿灯Y字路口右转
            d_veh2cross1(1)=max([traci.lane.getLength(':J34_2_0')+traci.lane.getLength('E36_0')-currentLanePosition 0]);    
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J34_2_0')+2 0]);
        elseif strcmp(current_road_ID,':J34_2')%无红绿灯Y字路口右转
            d_veh2cross1(1)=max([traci.lane.getLength(':J34_2_0')-currentLanePosition 0]);    
            d_veh2waitingArea=max([d_veh2cross1(1)-traci.lane.getLength(':J34_2_0')+2 0]);
        end
        %% 搜寻对向车
        if strcmp(current_road_ID,'E13') || strcmp(current_road_ID,':J12_4') || strcmp(current_road_ID,':J12_7')%Y字路口左转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E15_0',':J12_1_0','-E13_0',6.5);%45.6,13.14,
        elseif strcmp(current_road_ID,'E5') || strcmp(current_road_ID,':J6_9') || strcmp(current_road_ID,':J6_18')%十字路口左转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E6_0',':J6_1_0','-E5_0',10.8);%306.40,20.8,
        elseif strcmp(current_road_ID,'7') || strcmp(current_road_ID,':8_11') || strcmp(current_road_ID,':8_19')%十字路口左转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('9_1',':8_1_1','5_2',12.5); %132.40,20.8,
            [s_veh1(2),v_veh1(2),s_veh1apostrophe1(2),s_veh_length(2),s_veh_width(2),s_vehapostrophe_length(2),s_vehapostrophe_width(2)]=AvoOncomingEnvVehInform('9_0',':8_1_0','5_1',10.2);%132.40,20.8,
        elseif (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_7'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'E33')%无红绿灯路口直行
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('E32_0',':J30_10_0','E35_0',8.8);
            [s_veh1(2),v_veh1(2),s_veh1apostrophe1(2),s_veh_length(2),s_veh_width(2),s_vehapostrophe_length(2),s_vehapostrophe_width(2)]=AvoOncomingEnvVehInform('-E35_0',':J30_5_0',':J30_12_0',5.93);
            [s_veh1(3),v_veh1(3),s_veh1apostrophe1(3),s_veh_length(3),s_veh_width(3),s_vehapostrophe_length(3),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E33_0',':J30_2_0','E35_0',8.26);
            [s_veh1(4),v_veh1(4),s_veh1apostrophe1(4),s_veh_length(4),s_veh_width(4),s_vehapostrophe_length(4),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E35_0',':J30_4_0','-E32_0',5.6);
            [s_veh1(5),v_veh1(5),s_veh1apostrophe1(5),s_veh_length(5),s_veh_width(5),s_vehapostrophe_length(5),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E35_0',':J30_3_0','E33_0',9.03);
            [s_veh1(6),v_veh1(6),s_veh1apostrophe1(6),s_veh_length(6),s_veh_width(6),s_vehapostrophe_length(6),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('E32_0',':J30_11_0',':J30_13_0',14.19);
        elseif (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_8'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'-E32')%无红绿灯路口左转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('E32_0',':J30_10_0','E35_0',7.2);
            [s_veh1(2),v_veh1(2),s_veh1apostrophe1(2),s_veh_length(2),s_veh_width(2),s_vehapostrophe_length(2),s_vehapostrophe_width(2)]=AvoOncomingEnvVehInform('-E35_0',':J30_5_0',':J30_12_0',8.26);
            [s_veh1(3),v_veh1(3),s_veh1apostrophe1(3),s_veh_length(3),s_veh_width(3),s_vehapostrophe_length(3),s_vehapostrophe_width(3)]=AvoOncomingEnvVehInform('E32_0',':J30_11_0',':J30_13_0',5.93);
            [s_veh1(4),v_veh1(4),s_veh1apostrophe1(4),s_veh_length(4),s_veh_width(4),s_vehapostrophe_length(4),s_vehapostrophe_width(4)]=AvoOncomingEnvVehInform('-E33_0',':J30_1_0','E34_0',7.2);
            [s_veh1(5),v_veh1(5),s_veh1apostrophe1(5),s_veh_length(5),s_veh_width(5),s_vehapostrophe_length(5),s_vehapostrophe_width(5)]=AvoOncomingEnvVehInform('-E33_0',':J30_0_0','-E32_0',9.03);
            [s_veh1(6),v_veh1(6),s_veh1apostrophe1(6),s_veh_length(6),s_veh_width(6),s_vehapostrophe_length(6),s_vehapostrophe_width(6)]=AvoOncomingEnvVehInform('-E35_0',':J30_4_0','-E32_0',14.4);
        elseif (strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_6'))&&strcmp(route(find(strcmp(route,'-E34'))+1),'E35')&&usecase~=151&&usecase~=150%无红绿灯路口右转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('E32_0',':J30_10_0','E35_0',14.4);
            [s_veh1(2),v_veh1(2),s_veh1apostrophe1(2),s_veh_length(2),s_veh_width(2),s_vehapostrophe_length(2),s_vehapostrophe_width(2)]=AvoOncomingEnvVehInform('-E33_0',':J30_2_0','E35_0',14.19);
        elseif (strcmp(current_road_ID,'E36')||strcmp(current_road_ID,':J34_3')||strcmp(current_road_ID,':J34_6'))&&strcmp(route(find(strcmp(route,'E36'))+1),'E38')%无红绿灯Y路口左转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E38_0',':J34_5_0','E37_0',7.9);
            [s_veh1(2),v_veh1(2),s_veh1apostrophe1(2),s_veh_length(2),s_veh_width(2),s_vehapostrophe_length(2),s_vehapostrophe_width(2)]=AvoOncomingEnvVehInform('-E37_0',':J34_1_0','-E36_0',6.73);
            [s_veh1(3),v_veh1(3),s_veh1apostrophe1(3),s_veh_length(3),s_veh_width(3),s_vehapostrophe_length(3),s_vehapostrophe_width(3)]=AvoOncomingEnvVehInform('-E37_0',':J34_0_0','E38_0',9.03);
        elseif (strcmp(current_road_ID,'E36')||strcmp(current_road_ID,':J34_2'))&&strcmp(route(find(strcmp(route,'E36'))+1),'E37')%无红绿灯Y路口右转
            [s_veh1(1),v_veh1(1),s_veh1apostrophe1(1),s_veh_length(1),s_veh_width(1),s_vehapostrophe_length(1),s_vehapostrophe_width(1)]=AvoOncomingEnvVehInform('-E38_0',':J34_5_0','E37_0',14.19);
        end
      %% 停车让行停止线  
        if usecase>=145&&usecase<=151
            d_veh2Signstopline=min(d_veh2stopline,d_veh2waitingArea-6);
            d_veh2waitingArea=d_veh2waitingArea-6;
        elseif usecase==152
            d_veh2waitingArea=d_veh2waitingArea-0.5;
            d_veh2Signstopline=d_veh2waitingArea;
        else
            d_veh2Signstopline=200;
        end
        %% 搜寻掉头对向车
        t9=clock;
        if pos_s>-50&&pos_s<10&&(usecase<=31||(usecase>=168&&usecase<=175))%第一个路口
            oppositeVehiclesIDLane1=[traci.lane.getLastStepVehicleIDs(':8_1_1') traci.lane.getLastStepVehicleIDs('5_2') traci.lane.getLastStepVehicleIDs('9_1')];
            oppositeVehiclesIDLane2=[traci.lane.getLastStepVehicleIDs(':8_1_0') traci.lane.getLastStepVehicleIDs('5_1') traci.lane.getLastStepVehicleIDs('9_0')];
            oppositeVehiclesIDLane3_1=traci.lane.getLastStepVehicleIDs('5_0');
            oppositeVehiclesIDLane3_2=traci.lane.getLastStepVehicleIDs(':8_13_0');
            oppositeVehiclesIDLane3_3=traci.lane.getLastStepVehicleIDs('12_0');
            oppositeVehiclesIDLane1=oppositeVehiclesIDLane1(strcmp(oppositeVehiclesIDLane1,'S0')~=1);
            oppositeVehiclesIDLane2=oppositeVehiclesIDLane2(strcmp(oppositeVehiclesIDLane2,'S0')~=1);
            oppositeVehiclesIDLane3_1=oppositeVehiclesIDLane3_1(strcmp(oppositeVehiclesIDLane3_1,'S0')~=1);
            oppositeVehiclesIDLane3_2=oppositeVehiclesIDLane3_2(strcmp(oppositeVehiclesIDLane3_2,'S0')~=1);
            oppositeVehiclesIDLane3_3=oppositeVehiclesIDLane3_3(strcmp(oppositeVehiclesIDLane3_3,'S0')~=1);
            %Lane1--------------------------
            VehiclesLane1=VehicleInform1(oppositeVehiclesIDLane1,-6.4,-12.8,1);
            VehiclesLane2=VehicleInform1(oppositeVehiclesIDLane2,-6.4,-12.8,2);
            VehiclesLane3_1=VehicleInform1(oppositeVehiclesIDLane3_1,-6.4,-12.8,3);
            VehiclesLane3_2=VehicleInform3(oppositeVehiclesIDLane3_2,-6.4,-12.8,3,-10.6,9.03);
            VehiclesLane3_3=VehicleInform3(oppositeVehiclesIDLane3_3,-6.4,-12.8,3,-10.6+9.03,186.4);

            VehiclesLane3=[VehiclesLane3_1;VehiclesLane3_2;VehiclesLane3_3];
            VehiclesLane=[VehiclesLane1;VehiclesLane2;VehiclesLane3];
            if isempty(VehiclesLane)==0
                IndexOfLaneOppositeCar=VehiclesLane(:,1);
                SpeedOppositeCar=VehiclesLane(:,3);
                PosSOppositeCar=VehiclesLane(:,2);
                LengthOppositeCar=VehiclesLane(:,4);
            end
            
        elseif pos_s>100&&pos_s<160&&(usecase<=31||(usecase>=168&&usecase<=175))%第二个路口
            oppositeVehiclesIDLane1=[traci.lane.getLastStepVehicleIDs(':11_1_0') traci.lane.getLastStepVehicleIDs('-gneE0_0') traci.lane.getLastStepVehicleIDs('9_1')];
            oppositeVehiclesIDLane3_1=traci.lane.getLastStepVehicleIDs('9_0');
            oppositeVehiclesIDLane3_2=traci.lane.getLastStepVehicleIDs(':11_10_0');
            oppositeVehiclesIDLane3_3=traci.lane.getLastStepVehicleIDs('-gneE2_0');
            oppositeVehiclesIDLane1=oppositeVehiclesIDLane1(strcmp(oppositeVehiclesIDLane1,'S0')~=1);
            oppositeVehiclesIDLane3_1=oppositeVehiclesIDLane3_1(strcmp(oppositeVehiclesIDLane3_1,'S0')~=1);
            oppositeVehiclesIDLane3_2=oppositeVehiclesIDLane3_2(strcmp(oppositeVehiclesIDLane3_2,'S0')~=1);
            oppositeVehiclesIDLane3_3=oppositeVehiclesIDLane3_3(strcmp(oppositeVehiclesIDLane3_3,'S0')~=1);
            %Lane1--------------------------
            VehiclesLane1=VehicleInform1(oppositeVehiclesIDLane1,146.8,140.4,1);
            VehiclesLane3_1=VehicleInform1(oppositeVehiclesIDLane3_1,146.8,140.4,2);
            VehiclesLane3_2=VehicleInform3(oppositeVehiclesIDLane3_2,146.8,140.4,2,142.8,9.03);
            VehiclesLane3_3=VehicleInform3(oppositeVehiclesIDLane3_3,146.8,140.4,2,142.8+9.03,89.6);

            VehiclesLane3=[VehiclesLane3_1;VehiclesLane3_2;VehiclesLane3_3];
             VehiclesLane=[VehiclesLane1;VehiclesLane3];
%              VehiclesLane=VehiclesLane1;
            if isempty(VehiclesLane)==0
                IndexOfLaneOppositeCar=VehiclesLane(:,1);
                SpeedOppositeCar=VehiclesLane(:,3);
                PosSOppositeCar=VehiclesLane(:,2);
                LengthOppositeCar=VehiclesLane(:,4);
            end
        elseif pos_s>200&&pos_s<260&&(usecase<=31||(usecase>=168&&usecase<=175))%第三个路口
            oppositeVehiclesIDLane1=[traci.lane.getLastStepVehicleIDs(':gneJ0_1_0') traci.lane.getLastStepVehicleIDs('-gneE0_0') traci.lane.getLastStepVehicleIDs('gneE5_0')];
            oppositeVehiclesIDLane3_2=traci.lane.getLastStepVehicleIDs(':gneJ0_10_0');
            oppositeVehiclesIDLane3_3=traci.lane.getLastStepVehicleIDs('-gneE4_0');
            oppositeVehiclesIDLane1=oppositeVehiclesIDLane1(strcmp(oppositeVehiclesIDLane1,'S0')~=1);
            oppositeVehiclesIDLane3_2=oppositeVehiclesIDLane3_2(strcmp(oppositeVehiclesIDLane3_2,'S0')~=1);
            oppositeVehiclesIDLane3_3=oppositeVehiclesIDLane3_3(strcmp(oppositeVehiclesIDLane3_3,'S0')~=1);
            %Lane1--------------------------
            VehiclesLane1=VehicleInform1(oppositeVehiclesIDLane1,246.8,240.4,1);
            VehiclesLane3_2=VehicleInform3(oppositeVehiclesIDLane3_2,246.8,240.4,1,242.8,9.03);
            VehiclesLane3_3=VehicleInform3(oppositeVehiclesIDLane3_3,246.8,240.4,1,242.8+9.03,92.8);
            VehiclesLane3=[VehiclesLane3_2;VehiclesLane3_3];
            VehiclesLane=[VehiclesLane1;VehiclesLane3];
            if isempty(VehiclesLane)==0
                IndexOfLaneOppositeCar=VehiclesLane(:,1);
                SpeedOppositeCar=VehiclesLane(:,3);
                PosSOppositeCar=VehiclesLane(:,2);
                LengthOppositeCar=VehiclesLane(:,4);
            end
        end
        
        %% 搜寻掉头前车道环境车
        if pos_s>-50&&pos_s<10&&(usecase<=31||(usecase>=168&&usecase<=175))%第一个路口
            VehiclesIDLane1_1=[traci.lane.getLastStepVehicleIDs('7_1') traci.lane.getLastStepVehicleIDs('10_1') traci.lane.getLastStepVehicleIDs(':8_9_1')];
            VehiclesIDLane1_2=traci.lane.getLastStepVehicleIDs(':8_11_0');
            VehiclesIDLane1_3=traci.lane.getLastStepVehicleIDs(':8_19_0');
            VehiclesIDLane2=[traci.lane.getLastStepVehicleIDs('7_0') traci.lane.getLastStepVehicleIDs(':8_9_0')];
            VehiclesIDLane1_1=VehiclesIDLane1_1(strcmp(VehiclesIDLane1_1,'S0')~=1);
            VehiclesIDLane1_2=VehiclesIDLane1_2(strcmp(VehiclesIDLane1_2,'S0')~=1);
            VehiclesIDLane1_3=VehiclesIDLane1_3(strcmp(VehiclesIDLane1_3,'S0')~=1);
            %Lane1--------------------------
            VehicleInformLane1_1=VehicleInform1(VehiclesIDLane1_1,-6.4,-12.8,-1);
            VehicleInformLane1_2=VehicleInform2(VehiclesIDLane1_2,-6.4,-12.8,-1,-10.6);
            VehicleInformLane1_3=VehicleInform2(VehiclesIDLane1_3,-6.4,-12.8,-1,-10.6+6.41);

            %Lane2--------------------------
            VehicleInformLane2=VehicleInform1(VehiclesIDLane2,-6.4,-12.8,-2);
            VehicleInformLane1=[VehicleInformLane1_1;VehicleInformLane1_2;VehicleInformLane1_3];
            VehicleInformLane=[VehicleInformLane1;VehicleInformLane2];
            if isempty(VehicleInformLane)==0
                IndexOfLaneCodirectCar=VehicleInformLane(:,1);
                SpeedCodirectCar=VehicleInformLane(:,3);
                PosSCodirectCar=VehicleInformLane(:,2);
                LengthCodirectCar=VehicleInformLane(:,4);
            end
        elseif pos_s>100&&(usecase<=31||(usecase>=168&&usecase<=175))%第二个路口
            VehiclesIDLane1_1=traci.lane.getLastStepVehicleIDs('10_1');
            VehiclesIDLane1_2=traci.lane.getLastStepVehicleIDs(':11_8_0');
            VehiclesIDLane1_3=traci.lane.getLastStepVehicleIDs(':11_15_0');
            VehiclesIDLane2=[traci.lane.getLastStepVehicleIDs('10_0') traci.lane.getLastStepVehicleIDs('gneE0_0') traci.lane.getLastStepVehicleIDs(':11_7_0')];
            VehiclesIDLane1_1=VehiclesIDLane1_1(strcmp(VehiclesIDLane1_1,'S0')~=1);
            VehiclesIDLane1_2=VehiclesIDLane1_2(strcmp(VehiclesIDLane1_2,'S0')~=1);
            VehiclesIDLane1_3=VehiclesIDLane1_3(strcmp(VehiclesIDLane1_3,'S0')~=1);
            %Lane1--------------------------
             VehicleInformLane1_1=VehicleInform1(VehiclesIDLane1_1,146.8,140.4,-1);
             VehicleInformLane1_2=VehicleInform2(VehiclesIDLane1_2,146.8,140.4,-1,142.8);
             VehicleInformLane1_3=VehicleInform2(VehiclesIDLane1_2,146.8,140.4,-1,142.8+5.26);
            %lane2---------------------------------------------------------------------------------------------------------------------
            VehicleInformLane2=VehicleInform1(VehiclesIDLane2,146.8,140.4,-2);
            VehicleInformLane1=[VehicleInformLane1_1;VehicleInformLane1_2;VehicleInformLane1_3];
            VehicleInformLane=[VehicleInformLane1;VehicleInformLane2];
            if isempty(VehicleInformLane)==0
                IndexOfLaneCodirectCar=VehicleInformLane(:,1);
                SpeedCodirectCar=VehicleInformLane(:,3);
                PosSCodirectCar=VehicleInformLane(:,2);
                LengthCodirectCar=VehicleInformLane(:,4);
            end
        elseif pos_s>200&&pos_s<260&&(usecase<=31||(usecase>=168&&usecase<=175))%第三个路口
            VehiclesIDLane1_1=[traci.lane.getLastStepVehicleIDs('gneE0_0') traci.lane.getLastStepVehicleIDs('-gneE5_0') traci.lane.getLastStepVehicleIDs(':gneJ0_7_0')];
            VehiclesIDLane1_2=traci.lane.getLastStepVehicleIDs(':gneJ0_8_0');
            VehiclesIDLane1_3=traci.lane.getLastStepVehicleIDs(':gneJ0_15_0');
            VehiclesIDLane1_1=VehiclesIDLane1_1(strcmp(VehiclesIDLane1_1,'S0')~=1);
            VehiclesIDLane1_2=VehiclesIDLane1_2(strcmp(VehiclesIDLane1_2,'S0')~=1);
            VehiclesIDLane1_3=VehiclesIDLane1_3(strcmp(VehiclesIDLane1_3,'S0')~=1);
            %Lane1--------------------------
            VehicleInformLane1_1=VehicleInform1(VehiclesIDLane1_1,246.8,240.4,-1);
            VehicleInformLane1_2=VehicleInform2(VehiclesIDLane1_2,246.8,240.4,-1,242.8);
            VehicleInformLane1_3=VehicleInform2(VehiclesIDLane1_3,246.8,240.4,-1,242.8+5.93);
            VehiclesLane1=[VehicleInformLane1_1;VehicleInformLane1_2;VehicleInformLane1_3];
            if isempty(VehiclesLane1)==0
                SpeedCodirectCar=VehiclesLane1(:,3);
                PosSCodirectCar=VehiclesLane1(:,2);
                LengthCodirectCar=VehicleInformLane(:,4);
            end
        end
        t10=clock;
        %% 搜寻左右车
        if strcmp(current_road_ID,'E25')
            [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition);
%             NumOfLanes=6;
%             % 搜寻右车
%             lane=max([0 lane]);
%             lane=min([NumOfLanes-1 lane]);
%             RightLaneIndex=['E25_' num2str(round(max([0 lane-1])))];
%             vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(RightLaneIndex);
%             vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
%             vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
%             for id=1:length(vehicles_targetLaneIDs)
%                 vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
%                 vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
%             end
%             bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
%             cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
%             if isempty(bIDs)==0
%                 bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
%                 RightLaneFrontVel=vehicles_targetLaneSpeeds(bID);
%                 RightLaneFrontDis=vehicles_targetLanePoses(bID);
%             end
%             if isempty(cIDs)==0
%                 cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
%                 RightLaneBehindVel=vehicles_targetLaneSpeeds(cID);
%                 RightLaneBehindDis=vehicles_targetLanePoses(cID);
%             end
%             % 搜寻左车
%             LeftLaneIndex=['E25_' num2str(round(min([NumOfLanes-1 lane+1])))];
%             vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(LeftLaneIndex);
%             vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
%             vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
%             for id=1:length(vehicles_targetLaneIDs)
%                 vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
%                 vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
%             end
%             bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
%             cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
%             if isempty(bIDs)==0
%                 bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
%                 LeftLaneFrontVel=vehicles_targetLaneSpeeds(bID);
%                 LeftLaneFrontDis=vehicles_targetLanePoses(bID);
%             end
%             if isempty(cIDs)==0
%                 cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
%                 LeftLaneBehindVel=vehicles_targetLaneSpeeds(cID);
%                 LeftLaneBehindDis=vehicles_targetLanePoses(cID);
%             end
        elseif strcmp(current_road_ID,'E15')
            NumOfLanes=2;
            % 搜寻右车
            lane=max([0 lane]);
            lane=min([NumOfLanes-1 lane]);
            RightLaneIndex=['E15_' num2str(round(max([0 lane-1])))];
            vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(RightLaneIndex);
            vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
            vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
            for id=1:length(vehicles_targetLaneIDs)
                vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
                vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
            end
            bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
            cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
            if isempty(bIDs)==0
                bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
                RightLaneFrontVel=vehicles_targetLaneSpeeds(bID);
                RightLaneFrontDis=vehicles_targetLanePoses(bID);
            end
            if isempty(cIDs)==0
                cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
                RightLaneBehindVel=vehicles_targetLaneSpeeds(cID);
                RightLaneBehindDis=vehicles_targetLanePoses(cID);
            end
            % 搜寻左车
            LeftLaneIndex=['E15_' num2str(round(min([NumOfLanes-1 lane+1])))];
            vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(LeftLaneIndex);
            vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
            vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
            for id=1:length(vehicles_targetLaneIDs)
                vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
                vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
            end
            bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
            cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
            if isempty(bIDs)==0
                bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
                LeftLaneFrontVel=vehicles_targetLaneSpeeds(bID);
                LeftLaneFrontDis=vehicles_targetLanePoses(bID);
            end
            if isempty(cIDs)==0
                cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
                LeftLaneBehindVel=vehicles_targetLaneSpeeds(cID);
                LeftLaneBehindDis=vehicles_targetLanePoses(cID);
            end
        elseif strcmp(current_road_ID,'E12')
            [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
                ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition);
            if usecase==161
                LeftLaneFrontLen=5;
            end
        elseif strcmp(current_road_ID,'E10')
            [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition);
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
%             [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis]=...
%     LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition);
%            NumOfLanes=6;
            % 搜寻右车
            lane=max([0 lane]);
            lane=min([NumOfLanes-1 lane]);
            RightLaneIndex1=['A0_' num2str(round(max([0 lane-1])))];
            RightLaneIndex2=['A1_' num2str(round(max([0 lane-1])))];
            RightLaneIndex3=['A2_' num2str(round(max([0 lane-1])))];
            RightLaneIndex4=['A3_' num2str(round(max([0 lane-1])))];
            RightLaneIndex5=['A4_' num2str(round(max([0 lane-1])))];
            vehicles_targetLaneIDs=[traci.lane.getLastStepVehicleIDs(RightLaneIndex1) traci.lane.getLastStepVehicleIDs(RightLaneIndex2) traci.lane.getLastStepVehicleIDs(RightLaneIndex3) traci.lane.getLastStepVehicleIDs(RightLaneIndex4) traci.lane.getLastStepVehicleIDs(RightLaneIndex5)];
            vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
            vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
            for id=1:length(vehicles_targetLaneIDs)
                vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
                vehicles_targetLanePose=traci.vehicle.getPosition(vehicles_targetLaneIDs{id});
                vehicles_targetLanePoses(id)=vehicles_targetLanePose(1)-postion_veh(1);
            end
            bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
            cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
            if isempty(bIDs)==0
                bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
                RightLaneFrontVel=vehicles_targetLaneSpeeds(bID);
                RightLaneFrontDis=vehicles_targetLanePoses(bID);
            end
            if isempty(cIDs)==0
                cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
                RightLaneBehindVel=vehicles_targetLaneSpeeds(cID);
                RightLaneBehindDis=vehicles_targetLanePoses(cID);
            end
            % 搜寻左车
            LeftLaneIndex1=['A0_' num2str(round(min([NumOfLanes-1 lane+1])))];
            LeftLaneIndex2=['A1_' num2str(round(min([NumOfLanes-1 lane+1])))];
            LeftLaneIndex3=['A2_' num2str(round(min([NumOfLanes-1 lane+1])))];
            LeftLaneIndex4=['A3_' num2str(round(min([NumOfLanes-1 lane+1])))];
            LeftLaneIndex5=['A4_' num2str(round(min([NumOfLanes-1 lane+1])))];        
            vehicles_targetLaneIDs=[traci.lane.getLastStepVehicleIDs(LeftLaneIndex1) traci.lane.getLastStepVehicleIDs(LeftLaneIndex2) traci.lane.getLastStepVehicleIDs(LeftLaneIndex3) traci.lane.getLastStepVehicleIDs(LeftLaneIndex4) traci.lane.getLastStepVehicleIDs(LeftLaneIndex5)];
            vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
            vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
            for id=1:length(vehicles_targetLaneIDs)
                vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
                vehicles_targetLanePose=traci.vehicle.getPosition(vehicles_targetLaneIDs{id});
                vehicles_targetLanePoses(id)=vehicles_targetLanePose(1)-postion_veh(1);
            end
            bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
            cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
            if isempty(bIDs)==0
                bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
                LeftLaneFrontVel=vehicles_targetLaneSpeeds(bID);
                LeftLaneFrontDis=vehicles_targetLanePoses(bID);
            end
            if isempty(cIDs)==0
                cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
                LeftLaneBehindVel=vehicles_targetLaneSpeeds(cID);
                LeftLaneBehindDis=vehicles_targetLanePoses(cID);
            end
        end
       %% 搜寻不可通行车道
        if strcmp(current_road_ID,'E25')&&currentLanePosition>60&&(usecase<95||usecase==100||usecase>=206)
            for j=1:1:NumOfLanes
                SearchedLane=['E25_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                 LanesWithFail=zeros(1,6,'int16');
            elseif currentLanePosition>80&&(usecase==95||usecase==96)
                 LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&usecase==95
            for j=1:1:NumOfLanes
                SearchedLane=['E25_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>80
                 LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&(usecase==96||usecase==97)
            for j=1:1:NumOfLanes
                SearchedLane=['E25_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>150
                 LanesWithFail=zeros(1,6,'int16');
            end    
        elseif strcmp(current_road_ID,'E12')&&usecase<156
            for j=1:1:NumOfLanes
                SearchedLane=['E12_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                 LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E10')
            for j=1:1:NumOfLanes
                SearchedLane=['E10_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                 LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'A1')
            for j=1:1:NumOfLanes
                SearchedLane=['A1_' num2str(round(NumOfLanes-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                     LanesWithFail(j)=j;
                     LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                 LanesWithFail=zeros(1,6,'int16');
            end
        end

        %% 档位变换
        CurrentGear=Decision.TargetGear;
       %% 速度规划
        if strcmp(current_road_ID,'7')||strcmp(current_road_ID,':8_9')||strcmp(current_road_ID,':8_11')||strcmp(current_road_ID,':8_19')% && strcmp(route{length(route)-1},'7')
            % 掉头前直行
            if traci.trafficlights.getPhase('8')==0
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
            elseif traci.trafficlights.getPhase('8')==1
                greenLight=2;
                time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            % 需要给出：
            if strcmp(route(find(strcmp(route,'7'))+1),'5')
                if usecase==17||usecase==18||usecase==19||usecase==20||usecase==21||usecase==22||usecase==23||usecase==24||usecase==25||usecase==26
                    TrafficLightActive=int16(0);
                else
                    TrafficLightActive=int16(1);
                end
                TurnAroundActive=int16(1);
            elseif strcmp(route(find(strcmp(route,'7'))+1),'8')
                TrafficLightActive=int16(1);
                VehicleOncomingActive=int16(1);
            else
                if usecase<=31
                    TrafficLightActive=int16(0);
                else
                    TrafficLightActive=int16(1);
                end
            end
        elseif strcmp(current_road_ID,'10')%&& strcmp(route{length(route)-1},'10')
            % 掉头前直行
            if traci.trafficlights.getPhase('11')==0
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset2*10,900))/10;
            elseif traci.trafficlights.getPhase('11')==1
                greenLight=2;
                time2nextSwitch=(420-mod(i-TrafficLightOffset2*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            % 需要给出：
            if strcmp(route(find(strcmp(route,'10'))+1),'9')
                TrafficLightActive=int16(1);
                TurnAroundActive=int16(1);
            else
                if usecase<=31
                    TrafficLightActive=int16(0);
                else
                    TrafficLightActive=int16(1);
                end
            end
        elseif strcmp(current_road_ID,'gneE0')%&& strcmp(route{length(route)-1},'gneE0')
            % 掉头前直行
            if traci.trafficlights.getPhase('gneJ0')==0
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset3*10,900))/10;
            elseif traci.trafficlights.getPhase('gneJ0')==1
                greenLight=2;
                time2nextSwitch=(420-mod(i-TrafficLightOffset3*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            % 需要给出：
            if strcmp(route(find(strcmp(route,'gneE0'))+1),'-gneE0')
                TrafficLightActive=int16(1);
                TurnAroundActive=int16(1);
            else
                TrafficLightActive=int16(1);
            end
        elseif strcmp(current_road_ID,'9')||strcmp(current_road_ID,':8_0')%右转
            if traci.trafficlights.getPhase('8')==0
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
            elseif traci.trafficlights.getPhase('8')==1
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
            else
                greenLight=1;
                time2nextSwitch=420;
            end
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'8')%丁字路口右转
            if traci.trafficlights.getPhase('9')==2
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset4*10,900))/10;
            elseif traci.trafficlights.getPhase('9')==3
                greenLight=2;
                time2nextSwitch=(420-mod(i-TrafficLightOffset4*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            VehicleCrossingActive=int16(1);
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,':9_2')%丁字路口右转
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E13')
            if traci.trafficlights.getPhase('J12')==0
                greenLight=1;
                time2nextSwitch=(420-mod(i-TrafficLightOffset5*10,900))/10;
            elseif traci.trafficlights.getPhase('J12')==1
                greenLight=2;
                time2nextSwitch=(420-mod(i-TrafficLightOffset5*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            TrafficLightActive=int16(1);
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,':J12_4')||strcmp(current_road_ID,':J12_7')
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,'E1')||strcmp(current_road_ID,':J2_2')%穿行人行道
            PedestrianActive=int16(1);
        elseif strcmp(current_road_ID,'E2') || strcmp(current_road_ID,':J3_0')
            PedestrianActive=int16(1);
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E3') || strcmp(current_road_ID,':J4_1')
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>60
            if GlobVars.TrajPlanLaneChange.DurationLaneChange==0&&GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan==0
                if d_veh2goal<60
                    LaneChangeActive=int16(1);
                elseif TargetLaneIndex~=CurrentLaneIndex
                    LaneChangeActive=int16(1);
                else
                    LaneChangeActive=int16(0);
                end
            else
                LaneChangeActive=int16(1);
            end
            if LanesWithFail(1)~=0
                LaneChangeActive=int16(1);
            end
%             VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&(usecase==95||usecase==96||usecase==97||usecase==143)
            LaneChangeActive=int16(1);
%             VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E5')%J6路口前左转或右转
            if traci.trafficlights.getPhase('J6')==2
                greenLight=1;
                time2nextSwitch=(900-mod(i-TrafficLightOffset6*10,900))/10;
            elseif traci.trafficlights.getPhase('J6')==3
                greenLight=2;
                time2nextSwitch=(900-mod(i-TrafficLightOffset6*10,900))/10;
            else
                greenLight=0;
                time2nextSwitch=0;
            end
            if strcmp(route(find(strcmp(route,'E5'))+1),'E8')
                VehicleOncomingActive=int16(1);
                TrafficLightActive=int16(1);
            elseif strcmp(route(find(strcmp(route,'E5'))+1),'E7')
                VehicleCrossingActive=int16(1);    
            end
            PedestrianActive=int16(1);
        elseif strcmp(current_road_ID,':J6_9')||strcmp(current_road_ID,':J6_18')%J6路口中左转
            PedestrianActive=int16(1);
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,':J6_7')
            PedestrianActive=int16(1);
            VehicleCrossingActive=int16(1);
        elseif (usecase==101||usecase==102||usecase==103)&&strcmp(current_road_ID,'2')||strcmp(current_road_ID,':gneJ11_0')||strcmp(current_road_ID,':gneJ11_1')...
                ||strcmp(current_road_ID,'a8')||strcmp(current_road_ID,':gneJ13_2')||strcmp(current_road_ID,':gneJ13_3')||strcmp(current_road_ID,'12')||strcmp(current_road_ID,':gneJ14_2')
            VehicleCrossingActive=int16(1);
          pause(0.02)
        elseif strcmp(current_road_ID,'E15')||strcmp(current_road_ID,'E12')
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'E10')&&currentLanePosition>10
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_7')||strcmp(current_road_ID,':J30_8')||strcmp(current_road_ID,':J30_6')%无红绿灯路口直行或左转或右转
            if usecase>=150&&usecase<=151
                VehicleCrossingActive=int16(1);
            else
                VehicleOncomingActive=int16(1);
            end     
            PedestrianActive=int16(1);
        elseif strcmp(current_road_ID,'E36')||strcmp(current_road_ID,':J34_3')||strcmp(current_road_ID,':J34_6')||strcmp(current_road_ID,':J34_2')%无红绿灯Y字路口左转或右转
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'E40') || strcmp(current_road_ID,':J37_0')%三车道汇聚
            VehicleCrossingActive=int16(1); 
        elseif strcmp(current_road_ID,'E45') || strcmp(current_road_ID,':J43_0')%四车道汇聚
            VehicleCrossingActive=int16(1);  
            %         else
%             LaneChangeActive=0;
%             PedestrianActive=0;
%             TrafficLightActive=1;
%             VehicleCrossingActive=1;
%             VehicleOncomingActive=0;
        end
        %入参
        BasicsInfo.CurrentLaneFrontDis=CurrentLaneFrontDis;
        BasicsInfo.CurrentLaneFrontVel=CurrentLaneFrontVel;
        BasicsInfo.CurrentLaneFrontLen=CurrentLaneFrontLen;
        BasicsInfo.pos_s=pos_s;
        BasicsInfo.pos_l=pos_l;
        BasicsInfo.pos_l_CurrentLane=pos_l_CurrentLane;
        BasicsInfo.CurrentLaneIndex=int16(CurrentLaneIndex);
        BasicsInfo.WidthOfLanes=WidthOfLanes;
        BasicsInfo.TargetLaneIndex=int16(TargetLaneIndex);
        BasicsInfo.v_max=v_max;
        BasicsInfo.GoalLaneIndex=GoalLaneIndex;
        BasicsInfo.d_veh2goal=d_veh2goal;
        ChassisInfo.speed=double(speed);
        ChassisInfo.CurrentGear=int16(CurrentGear);
        AvoFailVehInfo.LanesWithFail=LanesWithFail;
        LaneChangeInfo.d_veh2int = d_veh2int;
        LaneChangeInfo.LeftLaneBehindDis=LeftLaneBehindDis;
        LaneChangeInfo.LeftLaneBehindVel=LeftLaneBehindVel;     
        LaneChangeInfo.LeftLaneFrontDis=LeftLaneFrontDis;
        LaneChangeInfo.LeftLaneFrontVel=LeftLaneFrontVel; 
        LaneChangeInfo.RightLaneBehindDis=RightLaneBehindDis;
        LaneChangeInfo.RightLaneBehindVel=RightLaneBehindVel;
        LaneChangeInfo.RightLaneFrontDis=RightLaneFrontDis;
        LaneChangeInfo.RightLaneFrontVel=RightLaneFrontVel;
        LaneChangeInfo.LeftLaneBehindLen=LeftLaneBehindLen;
        LaneChangeInfo.LeftLaneFrontLen=LeftLaneFrontLen;
        LaneChangeInfo.RightLaneBehindLen=RightLaneBehindLen;
        LaneChangeInfo.RightLaneFrontLen=RightLaneFrontLen;
        AvoMainRoVehInfo.CurrentLaneFrontDisAvoidVehicle=CurrentLaneFrontDisAvoidVehicle;
        AvoMainRoVehInfo.CurrentLaneFrontVelAvoidVehicle=CurrentLaneFrontVelAvoidVehicle;
        AvoMainRoVehInfo.CurrentLaneFrontLenAvoidVehicle=CurrentLaneFrontLenAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(1)=TargetLaneBehindDisAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(1)=TargetLaneBehindVelAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle(1)=TargetLaneBehindLenAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(1)=TargetLaneFrontDisAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(1)=TargetLaneFrontVelAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle(1)=TargetLaneFrontLenAvoidVehicle;
        AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(2)=TargetLaneBehindDisAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(2)=TargetLaneBehindVelAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle(2)=TargetLaneBehindLenAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(2)=TargetLaneFrontDisAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(2)=TargetLaneFrontVelAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle(2)=TargetLaneFrontLenAvoidVehicle1;
        AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(3)=TargetLaneBehindDisAvoidVehicle2;
        AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(3)=TargetLaneBehindVelAvoidVehicle2;
        AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle(3)=TargetLaneBehindLenAvoidVehicle2;
        AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(3)=TargetLaneFrontDisAvoidVehicle2;
        AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(3)=TargetLaneFrontVelAvoidVehicle2;
        AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle(3)=TargetLaneFrontLenAvoidVehicle2;
        AvoMainRoVehInfo.d_veh2converge=d_veh2converge;
        AvoMainRoVehInfo.d_veh2stopline=d_veh2stopline;
        StopSignInfo.d_veh2stopline=d_veh2Signstopline;
        AvoOncomingVehInfo.d_veh2waitingArea=d_veh2waitingArea;
        AvoOncomingVehInfo.s_veh=s_veh1;
        AvoOncomingVehInfo.v_veh=v_veh1;
        AvoOncomingVehInfo.s_veh_length=s_veh_length;
        AvoOncomingVehInfo.s_veh_width=s_veh_width;
        AvoOncomingVehInfo.d_veh2conflict=d_veh2cross1;
        AvoOncomingVehInfo.s_vehapostrophe=s_veh1apostrophe1;
        AvoOncomingVehInfo.s_vehapostrophe_length=s_vehapostrophe_length;
        AvoOncomingVehInfo.s_vehapostrophe_width=s_vehapostrophe_width;
        AvoPedInfo.d_veh2cross=d_veh2cross;
        AvoPedInfo.w_cross=w_cross;
        AvoPedInfo.s_ped=s_ped;
        AvoPedInfo.v_ped=v_ped;
        AvoPedInfo.l_ped=l_ped;
        AvoPedInfo.psi_ped=psi_ped;
        TrafficLightInfo.greenLight=greenLight;
        TrafficLightInfo.time2nextSwitch=time2nextSwitch;
        TrafficLightInfo.d_veh2stopline=d_veh2trafficStopline;
        TurnAroundInfo.NumOfLanesOpposite=int16(NumOfLanesOpposite);
        TurnAroundInfo.WidthOfLanesOpposite=WidthOfLanesOpposite;
        TurnAroundInfo.WidthOfGap=WidthOfGap;
        TurnAroundInfo.s_turnaround_border=s_turnaround_border;
        TurnAroundInfo.IndexOfLaneOppositeCar=IndexOfLaneOppositeCar;
        TurnAroundInfo.SpeedOppositeCar=SpeedOppositeCar;
        TurnAroundInfo.PosSOppositeCar=PosSOppositeCar;
        TurnAroundInfo.IndexOfLaneCodirectCar=IndexOfLaneCodirectCar;
        TurnAroundInfo.SpeedCodirectCar=SpeedCodirectCar;
        TurnAroundInfo.PosSCodirectCar=PosSCodirectCar;
        TurnAroundInfo.LengthOppositeCar=LengthOppositeCar;
        TurnAroundInfo.LengthCodirectCar=LengthCodirectCar;
        
        %         if manual~=1
        if StopSignInfo.d_veh2stopline<60 && StopSignInfo.d_veh2stopline>=1
            VehicleCrossingActive=0;
        end
        %% UrbanPlanner
        t2=clock;
        [Trajectory,Decision,GlobVars]=UrbanPlanner(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,...,
            AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,GlobVars,CalibrationVars,Parameters);
        t3=clock;
        % Decision
        if manual==1
            %鼠标---------------------------
            %             figure(1)
            %             if strcmp(get(gcf,'SelectionType'),'extend')
            %                 targetspeed=max(speed-0.3,0);
            %             elseif strcmp(get(gcf,'SelectionType'),'normal')
            %                 targetspeed=min(speed+0.15,12);
            %                 % 此时即为左键
            %             elseif strcmp(get(gcf,'SelectionType'),'alt')
            %                 % 此时即为右键
            %                 targetspeed=speed;
            %             end
            %             traci.vehicle.setSpeed('S0',targetspeed);
            %             pause(0.1)
            %---------------------------
            figure(1)
            traci.vehicle.changeLane('S0',lane,2);
            if strcmpi(get(gcf,'CurrentCharacter'),'d')
                targetspeed=max(speed-0.3,0);
            elseif strcmpi(get(gcf,'CurrentCharacter'),'e')
                targetspeed=min(speed+0.15,50/3.6);
            elseif strcmpi(get(gcf,'CurrentCharacter'),'r')
                targetspeed=speed;
            elseif strcmpi(get(gcf,'CurrentCharacter'),'s')
                traci.vehicle.changeLane('S0',min(lane+1,NumOfLanes-1),2);
                set(gcf,'CurrentCharacter','r')
            elseif strcmpi(get(gcf,'CurrentCharacter'),'f')
                traci.vehicle.changeLane('S0',max(lane-1,0),2);
                set(gcf,'CurrentCharacter','r')
            else
                targetspeed=speed;
            end
            traci.vehicle.setSpeed('S0',targetspeed);
            pause(0.01)
        elseif manual==2||manual==3
            %             if postion_veh(2)>=-240
            %                 traci.vehicle.setSpeedMode('S0', 31);
            %             else
            %                 traci.vehicle.setSpeedMode('S0', 0);
            %             end
            %             if usecase>=209&&usecase<=211
            %                 traci.vehicle.setSpeedMode('S0', 0);
            %             else
            %                 traci.vehicle.setSpeedMode('S0', 31);
            %             end
            traci.vehicle.setSpeedMode('S0', 31);
            if Decision.LaneChange==1
                traci.vehicle.changeLane('S0',lane+1,2);
            elseif Decision.LaneChange==2
                traci.vehicle.changeLane('S0',lane-1,2);
            else
                traci.vehicle.changeLane('S0',lane,2);
            end
            if Decision.SlowDown>0
                TimeResponse=TimeResponse+1;
                if TimeResponse>=10
                    if Decision.SlowDown==6
                        Fllowsts=1;
                        if speed>Decision.TargetSpeed/3.6
                            traci.vehicle.setSpeed('S0',speed-1.5*1);
                        else
                            traci.vehicle.setSpeed('S0',Decision.TargetSpeed/3.6);
                        end
                    else
                        traci.vehicle.setSpeed('S0',Decision.TargetSpeed/3.6);
                    end
                    TimeResponse=0;
                end
            elseif Decision.Wait>0
                traci.vehicle.setSpeedMode('S0', 0);
                %                 a_bre=min(-speed^2/((Decision.WaitDistance)*2),0);
                a_bre=(0-speed+(Decision.WaitDistance)/5)/1;
                a_bre=max(a_bre,-4);
                targetspeed=speed+a_bre*0.1;
                if Decision.WaitDistance<=0.2
                    targetspeed=max(0,speed+(-4.5)*0.1);
                    traci.vehicle.setSpeed('S0',targetspeed);
                else
                    traci.vehicle.setSpeed('S0',targetspeed);
                end
                if speed<1/3.6
                    StopVeh=1;
                end
            elseif StopVeh~=1
                if Fllowsts==1
                    TimeResponse1=TimeResponse1+1;
                    traci.vehicle.setSpeed('S0',min(50/3.6,CurrentLaneFrontVel));
                    if TimeResponse1>=10
                        Fllowsts=0;
                        TimeResponse1=0;
                    end
                else
                    TimeResponse=TimeResponse+1;
                    traci.vehicle.setSpeed('S0',speed);
                    if TimeResponse>=50
                        traci.vehicle.setSpeed('S0',50/3.6);
                        TimeResponse1=0;
                    end
                    
                end
            end
            if Decision.Start==1
                TimeResponse2=TimeResponse2+1;
                if TimeResponse2>=5&&usecase>=208&&usecase<=211&&manual~=3
                    traci.vehicle.setSpeed('S0',50/3.6);
                    StopVeh=0;
                    Fllowsts=0;
                    TimeResponse2=0;
                    TimeResponse=50;
                elseif TimeResponse2>=20
                    traci.vehicle.setSpeed('S0',50/3.6);
                    StopVeh=0;
                    Fllowsts=0;
                    TimeResponse2=0;
                    TimeResponse=50;
                end
            end
            if GlobVars.AEBDecision.AEBActive>0
                AEBdelay=AEBdelay+1;
                if AEBdelay>=2
                    targetspeed=max(0,speed+(-4)*0.1);
                    traci.vehicle.setSpeed('S0',targetspeed);
                end
            end
%             for traj_index=1:80
%                 traj_x=laneshape{1,1}(1,1)-Trajectory.traj_l(traj_index);
%                 traj_y=Trajectory.traj_s(traj_index);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x+0.1 traj_y+0.1]});
%             end
%            traci.vehicle.setSpeedMode('type3.15', 31); % 恢复考虑碰撞的速度模式 
        elseif Decision.a_soll~=100&&Decision.LaneChange==0
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
%             traci.vehicle.moveToXY('S0','E25', 2, laneshape{1,1}(1,1)-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
%             traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
            if usecase>31
                traci.vehicle.changeLane('S0',lane,2);
            end
            TimeResponse=0;
            
%             for traj_index=1:80
%                 traj_x=laneshape{1,1}(1,1)-Trajectory.traj_l(traj_index);
%                 traj_y=Trajectory.traj_s(traj_index);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x+0.1 traj_y+0.1]});
%             end
        elseif strcmp(current_road_ID,'E25')||strcmp(current_road_ID,'E12')||strcmp(current_road_ID,'E10')%避让故障车换道
            traci.vehicle.moveToXY('S0',current_road_ID, 2, laneshape{1,1}(1,1)-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
            
%             for traj_index=1:80
%                 traj_x=laneshape{1,1}(1,1)-Trajectory.traj_l(traj_index);
%                 traj_y=Trajectory.traj_s(traj_index);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x+0.1 traj_y+0.1]});
%             end   
        elseif strcmp(current_road_ID,'E15')%匝道汇出换道
            traci.vehicle.moveToXY('S0','E15', 2, -173.6-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
%             traci.vehicle.moveToXY('S0',current_road_ID, 2, Trajectory.traj_s(2), laneshape{1,1}(1,2)+Trajectory.traj_l(2),Trajectory.traj_psi(2),2);
%             traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
            if Decision.LaneChange==1
                TimeResponse=TimeResponse+1;
                if TimeResponse>=30
                   traci.vehicle.changeLane('S0',lane+1,0);
                   TimeResponse=0;
                end
            elseif Decision.LaneChange==2
                TimeResponse=TimeResponse+1;
                if TimeResponse>=30
                   traci.vehicle.changeLane('S0',lane-1,0);
                   TimeResponse=0;
                end
            else
                TimeResponse=0;
                traci.vehicle.changeLane('S0',lane,0);
            end
        else
            traci.vehicle.moveToXY('S0','12', 2, 101.6-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
%             for traj_index=1:80
%                 traj_x=101.6-Trajectory.traj_l(traj_index);
%                 traj_y=Trajectory.traj_s(traj_index);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x+0.1 traj_y+0.1]});
%             end
%             pause(0.5);
        end
        t4=clock;
        %                 if pos_s>70
        %                     pause(0.05);
        %                 end
        %         if Desider.wait_turnAround==1 || Desider.wait_TrafficLight==1 || Desider.wait_AvoidVehicle==1 || Desider.wait_ped==1|| Desider.wait_avoidOncomingVehicle==1
        %             sound(sin(2*pi*25*(1:4000)/100));
        %         end
        
        %画图
        if manual==2||(usecase>=201&&usecase<=205)
            w(1,i)=double(Decision.LaneChange);
            set(p(1),'XData',t,'YData',w(1,:))
            %         set(p(2),'XData',t,'YData',w(2,:))
            %         set(p(3),'XData',t,'YData',w(3,:))
            %         set(p(4),'XData',t,'YData',w(4,:))
            %         set(p(5),'XData',t,'YData',w(5,:))
            set(g,'String',['speed' 32 '=' 32 num2str(speed*3.6) 'km/h'],'Position',[x+12 2.3 0])
            set(wait_text,'String',['wait' 32 '=' 32 num2str(Decision.Wait)],'Position',[x+1 2.6 0])
            if Decision.WaitDistance>10
                set(waitDist_text,'String',['dist' 32 32 '=' 32 'NaN'],'Position',[x+1 2.3 0])
            else
                set(waitDist_text,'String',['dist' 32 32 '=' 32 num2str(Decision.WaitDistance) 'm'],'Position',[x+1 2.3 0])
            end
            set(slowdown_text,'String',['slowd' 32 32 '=' 32 num2str(Decision.SlowDown)],'Position',[x+12 2.9 0])
            if Decision.TargetSpeed==-20
                set(targetspeed_text,'String',['targV' 32 32 '=' 32 'NaN'],'Position',[x+12 2.6 0])
            else
                set(targetspeed_text,'String',['targV' 32 32 '=' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+12 2.6 0])
            end
            set(start_text,'String',['start' 32 '=' 32 num2str(Decision.Start)],'Position',[x+1 2.9 0])
            set(AEB_text,'String',['AEB' 32 '=' 32 num2str(Decision.AEBactive)],'Position',[x+30 2.3 0])
            set(Title,'String',['TargetLane=' num2str(TargetLaneIndex) 32 32 32 32 'CurrentLane=' num2str(CurrentLaneIndex)]);
            x=x+0.1;
            axis([x x+50 -0.5 3]);
            drawnow
        elseif manual==3&&i<=900
            pause(0.06)
            if Decision.LaneChange==1
                set(g,'String','请向左换道','Position',[x+15 ypos 0])
            elseif Decision.LaneChange==2
                set(g,'String','请向右换道','Position',[x+15 ypos 0])
            elseif Decision.SlowDown==1
                set(g,'String',['注意行人，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==2
                set(g,'String',['注意同向车辆，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==3
%                 set(g,'String',['注意对向车，请减速' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+6 ypos 0])
            elseif Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==6
                set(g,'String',['注意前车，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==7
                set(g,'String',['已超速，请减速' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+6 ypos 0])
            elseif (Decision.Wait==1||Decision.Wait==2||Decision.Wait==4)&&speed>0.5
                set(g,'String','请在停止线前停车','Position',[x+13 ypos 0])
            elseif Decision.Wait==3&&speed>0.2
                set(g,'String','请在待转区停车','Position',[x+14 ypos 0])
            elseif Decision.Wait==6
                set(g,'String','请在前车后方停车','Position',[x+13 ypos 0])
            elseif Decision.Start==1
                set(g,'String','请起步','Position',[x+18 ypos 0])
            else
                set(g,'String',' ')
            end
            drawnow
        elseif manual==3&&i<=1100
                        pause(0.06)
            if Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif (Decision.Wait==1||Decision.Wait==2||Decision.Wait==4)&&speed>0.5
                set(g,'String','请在停止线前停车','Position',[x+13 ypos 0])
            elseif Decision.Wait==3&&speed>0.2
                set(g,'String','请在待转区停车','Position',[x+14 ypos 0])
            end
            drawnow
        elseif manual==3
                       pause(0.06)
            if Decision.LaneChange==1
                set(g,'String','请向左换道','Position',[x+15 ypos 0])
            elseif Decision.LaneChange==2
                set(g,'String','请向右换道','Position',[x+15 ypos 0])
            elseif Decision.SlowDown==1
                set(g,'String',['注意行人，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==2
                set(g,'String',['注意同向车辆，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==3
%                 set(g,'String',['注意对向车，请减速' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+6 ypos 0])
            elseif Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==6
                set(g,'String',['注意前车，请减速至' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==7
                set(g,'String',['已超速，请减速' 32 num2str(Decision.TargetSpeed) 'km/h'],'Position',[x+6 ypos 0])
            elseif (Decision.Wait==1||Decision.Wait==2||Decision.Wait==4)&&speed>0.5
                set(g,'String','请在停止线前停车','Position',[x+13 ypos 0])
            elseif Decision.Wait==3&&speed>0.2
                if i>=1200&&i<=1230
                    set(g,'String','请起步','Position',[x+18 ypos 0])
                else
                set(g,'String','请在待转区停车','Position',[x+14 ypos 0])
                end
            elseif Decision.Wait==6
                set(g,'String','请在前车后方停车','Position',[x+13 ypos 0])
            elseif Decision.Start==1
                set(g,'String','请起步','Position',[x+18 ypos 0])
            else
                set(g,'String',' ')
            end
           if i>=1200&&i<=1230
                    set(g,'String','请起步','Position',[x+18 ypos 0])
           end
            drawnow 
        end
%-----------------------------------------------------------------------------------------------------------------------------------------------------------
        %% 仿真
        if i>794&&usecase==17
            traci.vehicle.setSpeedMode('type4.4', 0);
            traci.vehicle.setSpeed('type4.4',9);
        elseif i>798&&usecase==19
            traci.vehicle.setSpeedMode('type4.4', 0);
            traci.vehicle.setSpeed('type4.4',15);
        elseif i>960&&usecase==20
            traci.vehicle.setSpeedMode('type4.7', 0);
            traci.vehicle.setSpeed('type4.7',9);
        elseif i>1080&&usecase==21
            traci.vehicle.setSpeedMode('type5.4', 0);
            traci.vehicle.setSpeed('type5.4',12);
        end
        if i>750&&usecase==18
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','12', 2, 98.4, postion(2)+0.34,180);
        elseif i>1065&&usecase==22
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 1, 95.2, postion(2)+0.8,180,0);
        elseif i>935&&usecase==23
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 2, 98.4, postion(2)+0.8,180,0);
        elseif i>815&&usecase==24
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 1, 98.4, postion(2)+1,180,0);
        end
        if usecase==25
            if i>950&&i<1000
                postion=traci.vehicle.getPosition('S1');
                traci.vehicle.moveToXY('S1','7', 1, 101.6, postion(2)+2,0,2);
            else
                traci.vehicle.setSpeed('S1',0);
            end
        elseif usecase==26
            if i>960&&i<1000
                postion=traci.vehicle.getPosition('S1');
                traci.vehicle.moveToXY('S1','7', 1, 101.6, postion(2)-1,0,2);
            end
        elseif usecase==41
            if i>830&&i<900
                traci.vehicle.setSpeedMode('type24.0', 0);
                traci.vehicle.setSpeed('type24.0',20)
            end
        elseif usecase==233
            if i>830&&i<900
                traci.vehicle.setSpeedMode('type24.0', 0);
                traci.vehicle.setSpeed('type24.0',20)
            end
        elseif usecase==51
            if i>740 && s_ped(1)<800
                traci.person.setSpeed('p2',0)
            end
        elseif usecase==52
            if i>760 && s_ped(1)<800 && i<850
                traci.person.setSpeed('p2',0)
            elseif i==850
                traci.person.setSpeed('p2',3)
            end
        elseif usecase==60
            if i==790
                traci.trafficlights.setPhase('8',2);
            end
        elseif usecase==235
            if i==795
                traci.trafficlights.setPhase('8',2);
            end
        elseif usecase==69||usecase==45
            pause(0.05)
        elseif usecase==75
            if i>960&&i<1000
                traci.vehicle.setSpeedMode('type8.7', 0);
                traci.vehicle.setSpeed('type8.7',10)
            end
        elseif usecase==234
            if i>=900
                traci.vehicle.setColor ('type8.7',[255,255,255,255])
            end
            if i>970&&i<1000
                traci.vehicle.setSpeedMode('type8.7', 0);
                traci.vehicle.setSpeed('type8.7',10)
            end
            if i>920&&i<970
                traci.vehicle.setSpeed('type2.5',0)
            end
        elseif usecase==76
            if i>790&&i<810
                traci.vehicle.setSpeedMode('type9.0', 0);
                traci.vehicle.setSpeed('type9.0',2)
            elseif i>810&&i<925
                traci.vehicle.setSpeedMode('type9.0', 0);
                traci.vehicle.setSpeed('type9.0',0)
            elseif i>925&&i<950
                traci.vehicle.setSpeedMode('type9.0', 0);
                traci.vehicle.setSpeed('type9.0',2)
            end
        elseif usecase==61
            if i>716&&i<750
                traci.vehicle.setSpeedMode('type10.0', 0);
                traci.vehicle.setSpeed('type10.0',20)
            end
            pause(0.05)
        elseif usecase==62
            if i>716&&i<900
                traci.vehicle.changeLane('type10.0',2,2);
            end
            pause(0.05)
        elseif usecase==63
            if i>715&&i<900
                traci.vehicle.setSpeedMode('type10.0', 0);
                traci.vehicle.setSpeed('type10.0',6)
            end
            pause(0.05)
        elseif usecase==102||usecase==103
            if i>1160&&i<1280
                traci.vehicle.setSpeedMode('type11.0', 0);
                traci.vehicle.setSpeedMode('type12.0', 0);
                traci.vehicle.setSpeedMode('type13.0', 0);
                traci.vehicle.setSpeed('type11.0',8.3)
                traci.vehicle.setSpeed('type12.0',8.3)
                traci.vehicle.setSpeed('type13.0',8.3)
            end
            if usecase==103
                if i>1350&&i<1400
                    traci.vehicle.setSpeedMode('type8.20', 0);
                    traci.vehicle.setSpeedMode('type9.20', 0);
                    traci.vehicle.setSpeedMode('type8.21', 0);
                    traci.vehicle.setSpeedMode('type9.21', 0);
                    traci.vehicle.setSpeedMode('type8.22', 0);
                    traci.vehicle.setSpeed('type8.20',6)
                    traci.vehicle.setSpeed('type9.20',6)
                    traci.vehicle.setSpeed('type8.21',6)
                    traci.vehicle.setSpeed('type9.21',6)
                    traci.vehicle.setSpeed('type8.22',6)
                end
            end
        elseif usecase==210
            if i>780&&i<850
                traci.vehicle.setSpeedMode('type1.0', 0);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i>850
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',13.8)
            end
        elseif usecase==211
            if i==730
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==800
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            elseif i==850
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==920
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            elseif i==970
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==1040
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            end
        elseif usecase==129||usecase==232
            if i==730
                traci.person.setSpeed('p5.0',0)
            end
        elseif usecase==130
            if i==770
                traci.person.setSpeed('p6.0',0)
            end
        elseif usecase==216
            if i==730
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',50/3.6)
            elseif i==750
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',40/3.6)
            elseif i==770
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',30/3.6)
            elseif i==790
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',20/3.6)
            elseif i==810
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10/3.6)
            end
        elseif usecase==155
            if i==760
                traci.vehicle.setSpeed('type5.0',0);
            elseif i==930
                traci.vehicle.setSpeed('type5.0',30/3.6);
            end
            
        elseif usecase==206
            if i==1320
                traci.vehicle.setSpeed('type9.16',0);
            elseif i==1440
                traci.vehicle.setSpeed('type9.16',30/3.6);
            end
        elseif usecase==172
            if i==1110
                traci.vehicle.setSpeed('type44.0',0);
            elseif i==1200
                traci.vehicle.setSpeed('type44.0',30/3.6);
            end
            %         elseif usecase==153
            %             if i==730
            %                 traci.vehicle.setSpeedMode('type1.0', 31);
            %                 traci.vehicle.setSpeed('type1.0',50/3.6)
            %             elseif i==750
            %                 traci.vehicle.setSpeedMode('type1.0', 31);
            %                 traci.vehicle.setSpeed('type1.0',40/3.6)
            %             elseif i==770
            %                 traci.vehicle.setSpeedMode('type1.0', 31);
            %                 traci.vehicle.setSpeed('type1.0',30/3.6)
            %             elseif i==790
            %                 traci.vehicle.setSpeedMode('type1.0', 31);
            %                 traci.vehicle.setSpeed('type1.0',20/3.6)
            %             elseif i==810
            %                 traci.vehicle.setSpeedMode('type1.0', 31);
            %                 traci.vehicle.setSpeed('type1.0',10/3.6)
            %             end
            
            
        end
        if Draw==1
            T1=etime(t2,t1);
            T2=etime(t3,t2);
            T3=etime(t4,t3);
            T4=etime(t5,t1);
            T5=etime(t6,t5);
            T6=etime(t7,t6);
            T7=etime(t8,t7);
            T8=etime(t9,t8);
            T9=etime(t10,t9);
            T10=etime(t2,t10);
            w(1,i)=double(T1);
            w(2,i)=double(T2);
            w(3,i)=double(T3);
            w(4,i)=double(T4);
            w(5,i)=double(T5);
            w(6,i)=double(T6);
            w(7,i)=double(T7);
            w(8,i)=double(T8);
            w(9,i)=double(T9);
            w(10,i)=double(T10);            
            set(p(1),'XData',t,'YData',w(1,:))
            set(p(2),'XData',t,'YData',w(2,:))
            set(p(3),'XData',t,'YData',w(3,:))
            set(p(4),'XData',t,'YData',w(4,:))
            set(p(5),'XData',t,'YData',w(5,:))
            set(p(6),'XData',t,'YData',w(6,:))
            set(p(7),'XData',t,'YData',w(7,:))
            set(p(8),'XData',t,'YData',w(8,:))
            set(p(9),'XData',t,'YData',w(9,:))
            set(p(10),'XData',t,'YData',w(10,:))
            x=x+0.1;
            axis([x x+50 -0.2 0.5]);
            drawnow
        end
    end
    if usecase==30
        if i>510&&i<=550
            traci.vehicle.setSpeed('S1',0);
        elseif i>950&&i<=1100
            traci.vehicle.setSpeed('S1',6);
        end
    elseif usecase==31
        if i>510&&i<=550
            traci.vehicle.setSpeed('S1',0);
        elseif i>1050&&i<=1100
            traci.vehicle.setSpeed('S1',6);
        end
    elseif usecase==171
        if i>510&&i<=550
            traci.vehicle.setSpeed('S1',0);
        elseif i>1030&&i<=1100
            traci.vehicle.setSpeed('S1',6);
        end
        if i>510&&i<=1100
           traci.vehicle.changeLane('S1',1,2);
        end
    elseif usecase==173
        if i>510&&i<=590
            traci.vehicle.setSpeed('S1',0);
        elseif i>1150&&i<=1200
            traci.vehicle.setSpeed('S1',6);
        end
    elseif usecase==43||usecase==44||usecase==42
        if i>500
           traci.vehicle.changeLane('type10.0',3,2);
        end
    elseif usecase==46
        if i>600
            traci.vehicle.changeLane('type10.0',1.5,2);
        end
    elseif usecase==79
        if i>500
            PositionCars=traci.vehicle.getLanePosition('type2.0');
            if PositionCars>95
                traci.vehicle.setSpeed('type2.0',0.09);
                traci.vehicle.changeLane('type2.0',1,2);
            end
        end 
    end
end
traci.close();
 %-----------------------------------------------------------------------------------------------------------------------------------------------------------
%% 函数
 %搜直道车信息
function [VehicleLaneSSpeed]=VehicleInform1(VehiclesIDLane,s_turnaround_border,pos_start,Lane)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),3);%坐标、速度、车长
    for id=1:length(VehiclesIDLane)
        Position=traci.vehicle.getPosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=Position(2);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
        VehiclesLane(id,3)=traci.vehicle.getLength(VehiclesIDLane{id});
    end
    %FrontVehicle
    VehiclesFrontLane=VehiclesLane(VehiclesLane(:,1)>s_turnaround_border,:);
    FrontvehicleIDidmin=VehiclesFrontLane(:,1)==min(VehiclesFrontLane(:,1));
    VehicleFrontLane=VehiclesFrontLane(FrontvehicleIDidmin,:);
    %MidVehicle
    VehiclesMidslowLane=VehiclesLane(VehiclesLane(:,1)<=s_turnaround_border,:);
    VehiclesMidLane=VehiclesMidslowLane(VehiclesMidslowLane(:,1)>=pos_start,:);
    %RearVehicle
    VehiclesRearLane=VehiclesLane(VehiclesLane(:,1)<pos_start,:);
    RearvehicleIDidmin=VehiclesRearLane(:,1)==max(VehiclesRearLane(:,1));
    VehicleRearLane=VehiclesRearLane(RearvehicleIDidmin,:);
    %allVehicle
    Vehicles2Lane=[VehicleFrontLane;VehiclesMidLane;VehicleRearLane];
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),4);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:4)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end
function [VehicleLaneSSpeed]=VehicleInform2(VehiclesIDLane,s_turnaround_border,pos_start,Lane,LaneRefPos)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),3);
    for id=1:length(VehiclesIDLane)
        VehiclesLane(id,1)=traci.vehicle.getLanePosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=LaneRefPos+VehiclesLane(id,1);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
        VehiclesLane(id,3)=traci.vehicle.getLength(VehiclesIDLane{id});
    end
    %FrontVehicle
    VehiclesFrontLane=VehiclesLane(VehiclesLane(:,1)>s_turnaround_border,:);
    FrontvehicleIDidmin=VehiclesFrontLane(:,1)==min(VehiclesFrontLane(:,1));
    VehicleFrontLane=VehiclesFrontLane(FrontvehicleIDidmin,:);
    %MidVehicle
    VehiclesMidslowLane=VehiclesLane(VehiclesLane(:,1)<=s_turnaround_border,:);
    VehiclesMidLane=VehiclesMidslowLane(VehiclesMidslowLane(:,1)>=pos_start,:);
    %rearVehicle
    VehiclesRearLane=VehiclesLane(VehiclesLane(:,1)<pos_start,:);
    RearvehicleIDidmin=VehiclesRearLane(:,1)==max(VehiclesRearLane(:,1));
    VehicleRearLane=VehiclesRearLane(RearvehicleIDidmin,:);
    %allVehicle
    Vehicles2Lane=[VehicleFrontLane;VehiclesMidLane;VehicleRearLane];
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),4);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:4)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end
function [VehicleLaneSSpeed]=VehicleInform3(VehiclesIDLane,s_turnaround_border,pos_start,Lane,LaneRefPos,Lanelength)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),3);
    for id=1:length(VehiclesIDLane)
        VehiclesLane(id,1)=traci.vehicle.getLanePosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=LaneRefPos+Lanelength-VehiclesLane(id,1);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
        VehiclesLane(id,3)=traci.vehicle.getLength(VehiclesIDLane{id});
    end
    %FrontVehicle
    VehiclesFrontLane=VehiclesLane(VehiclesLane(:,1)>s_turnaround_border,:);
    FrontvehicleIDidmin=VehiclesFrontLane(:,1)==min(VehiclesFrontLane(:,1));
    VehicleFrontLane=VehiclesFrontLane(FrontvehicleIDidmin,:);
    %MidVehicle
    VehiclesMidslowLane=VehiclesLane(VehiclesLane(:,1)<=s_turnaround_border,:);
    VehiclesMidLane=VehiclesMidslowLane(VehiclesMidslowLane(:,1)>=pos_start,:);
    %rearVehicle
    VehiclesRearLane=VehiclesLane(VehiclesLane(:,1)<pos_start,:);
    RearvehicleIDidmin=VehiclesRearLane(:,1)==max(VehiclesRearLane(:,1));
    VehicleRearLane=VehiclesRearLane(RearvehicleIDidmin,:);
    %allVehicle
    Vehicles2Lane=[VehicleFrontLane;VehiclesMidLane;VehicleRearLane];
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),4);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:4)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end
% function [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform(vehicles_targetLaneIDs,converge,d_veh2converge,axis,direction)
% vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
% vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
% for id=1:length(vehicles_targetLaneIDs)
%     vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
%     postion=traci.vehicle.getPosition(vehicles_targetLaneIDs{id});
%     if strcmp(axis,'X')&&strcmp(direction,'-')
%         vehicles_targetLanePoses(id)=d_veh2converge-(postion(1)-converge);
%     elseif strcmp(axis,'X')&&strcmp(direction,'+')
%         vehicles_targetLanePoses(id)=d_veh2converge-(converge-postion(1));
%     elseif strcmp(axis,'Y')&&strcmp(direction,'-')
%         vehicles_targetLanePoses(id)=d_veh2converge-(postion(2)-converge);
%     else
%         vehicles_targetLanePoses(id)=d_veh2converge-(converge-postion(2));  
%     end
% end
% dIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
% eIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
% if isempty(dIDs)==0
%     dID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
%     v_f=vehicles_targetLaneSpeeds(dID);
%     s_f=vehicles_targetLanePoses(dID);
% else
%     v_f=20;
%     s_f=200;
% end
% if isempty(eIDs)==0
%     eID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
%     v_r=vehicles_targetLaneSpeeds(eID);
%     s_r=vehicles_targetLanePoses(eID);
% else 
%     v_r=20;
%     s_r=-200;
% end
% end
function [s_r,v_r,s_f,v_f]=AvoMainRoEnvVehInform2(RoadlaneID1,RoadlaneID2,RoadlaneID3,lenRoad1,lenRoad2,disroad2converge,d_veh2converge)%搜同向车辆
oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs(RoadlaneID1) traci.lane.getLastStepVehicleIDs(RoadlaneID2) traci.lane.getLastStepVehicleIDs(RoadlaneID3)];
oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
for id=1:length(oncomingVehiclesIDs)
    oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id}); 
    oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
    if strcmp(oncomingVehicleRoadLaneID,RoadlaneID1)
        oncomingVehiclesPositions(id)=d_veh2converge-(lenRoad1+disroad2converge-oncomingVehiclePosition);
    elseif strcmp(oncomingVehicleRoadLaneID,RoadlaneID2)
        oncomingVehiclesPositions(id)=d_veh2converge-(disroad2converge-oncomingVehiclePosition);
    elseif strcmp(oncomingVehicleRoadLaneID,RoadlaneID3)
        oncomingVehiclesPositions(id)=d_veh2converge-(-(lenRoad2-disroad2converge)-oncomingVehiclePosition);
    end
    oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
end
if ~isempty(oncomingVehiclesIDs)
    veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>0);
    veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<=0);
    if isempty(veh1IDs)==0
        veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>0)));
        s_f=oncomingVehiclesPositions(veh1ID);
        v_f=oncomingVehiclesSpeeds(veh1ID);
    else
        s_f=200;
        v_f=20;
    end
    if isempty(veh1apostrophe1IDs)==0
        veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
        s_r=oncomingVehiclesPositions(veh1apostrophe1ID);
        v_r=oncomingVehiclesSpeeds(veh1apostrophe1ID);
    else
        s_r=-200;
        v_r=20;
    end
else
    s_f=200;
    v_f=20;
    s_r=-200;
    v_r=20;
end
end
function [s_r,v_r,l_r,s_f,v_f,l_f,s_a,v_a,l_a]=AvoMainRoEnvVehInform3(RoadlaneID1,RoadlaneID2,RoadlaneID3,lenRoad1,lenRoad2,disroad2converge,d_veh2converge)%搜同向车辆
VehiclesIDs=[traci.lane.getLastStepVehicleIDs(RoadlaneID1) traci.lane.getLastStepVehicleIDs(RoadlaneID2) traci.lane.getLastStepVehicleIDs(RoadlaneID3)];
VehiclesPositions=zeros(length(VehiclesIDs),1);
VehiclesSpeeds=zeros(length(VehiclesIDs),1);
VehiclesLengths=zeros(length(VehiclesIDs),1);
for id=1:length(VehiclesIDs)
    VehiclePosition=traci.vehicle.getLanePosition(VehiclesIDs{id}); 
    VehicleRoadLaneID=traci.vehicle.getLaneID(VehiclesIDs{id});
    if strcmp(VehicleRoadLaneID,RoadlaneID1)
        VehiclesPositions(id)=d_veh2converge-(lenRoad1+disroad2converge-VehiclePosition);
    elseif strcmp(VehicleRoadLaneID,RoadlaneID2)
        VehiclesPositions(id)=d_veh2converge-(disroad2converge-VehiclePosition);
    elseif strcmp(VehicleRoadLaneID,RoadlaneID3)
        VehiclesPositions(id)=d_veh2converge-(-(lenRoad2-disroad2converge)-VehiclePosition);
    end
    VehiclesSpeeds(id)=traci.vehicle.getSpeed(VehiclesIDs{id});
    VehiclesLengths(id)=traci.vehicle.getLength(VehiclesIDs{id});
end
if ~isempty(VehiclesIDs)
    veh1IDs=VehiclesPositions(VehiclesPositions>0&VehiclesPositions<d_veh2converge);
    veh1apostrophe1IDs=VehiclesPositions(VehiclesPositions<=0);
    veh2IDs=VehiclesPositions(VehiclesPositions>=d_veh2converge);
    if isempty(veh1IDs)==0
        veh1ID=find(VehiclesPositions==min(VehiclesPositions(VehiclesPositions>0&VehiclesPositions<d_veh2converge)));
        s_f=VehiclesPositions(veh1ID);
        v_f=VehiclesSpeeds(veh1ID);
        l_f=VehiclesLengths(veh1ID);
    else
        s_f=200;
        v_f=20;
        l_f=5;
    end
    if isempty(veh1apostrophe1IDs)==0
        veh1apostrophe1ID=VehiclesPositions==max(VehiclesPositions(VehiclesPositions<0));
        s_r=VehiclesPositions(veh1apostrophe1ID);
        v_r=VehiclesSpeeds(veh1apostrophe1ID);
        l_r=VehiclesLengths(veh1apostrophe1ID);
    else
        s_r=-200;
        v_r=20;
        l_r=5;
    end
    if isempty(veh2IDs)==0
        veh1ID=find(VehiclesPositions==min(VehiclesPositions(VehiclesPositions>=d_veh2converge)));
        s_a=VehiclesPositions(veh1ID);
        v_a=VehiclesSpeeds(veh1ID);
        l_a=VehiclesLengths(veh1ID);
    else
        s_a=200;
        v_a=20;
        l_a=5;
    end
else
    s_f=200;
    v_f=20;
    s_r=-200;
    v_r=20;
    s_a=200;
    v_a=20;
    l_f=5;
    l_r=5;
    l_a=5;
end
end
function [s_rampf,v_rampf,l_rampf]=RampVehInform3(RoadlaneID1,RoadlaneID2,d_veh2converge)%
lenRoad1=traci.lane.getLength(RoadlaneID1);
lenRoad2=traci.lane.getLength(RoadlaneID2);
VehiclesIDs=[traci.lane.getLastStepVehicleIDs(RoadlaneID1) traci.lane.getLastStepVehicleIDs(RoadlaneID2)];
VehiclesIDs=VehiclesIDs(~strcmp(VehiclesIDs,'S0'));
VehiclesPositions=zeros(length(VehiclesIDs),1);
VehiclesSpeeds=zeros(length(VehiclesIDs),1);
VehiclesLengths=zeros(length(VehiclesIDs),1);
for id=1:length(VehiclesIDs)
    oncomingVehiclePosition=traci.vehicle.getLanePosition(VehiclesIDs{id}); 
    oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(VehiclesIDs{id});
    if strcmp(oncomingVehicleRoadLaneID,RoadlaneID1)
        VehiclesPositions(id)=d_veh2converge-(lenRoad1+lenRoad2-oncomingVehiclePosition);
    elseif strcmp(oncomingVehicleRoadLaneID,RoadlaneID2)
        VehiclesPositions(id)=d_veh2converge-(lenRoad2-oncomingVehiclePosition);
    end
    VehiclesSpeeds(id)=traci.vehicle.getSpeed(VehiclesIDs{id});
    VehiclesLengths(id)=traci.vehicle.getLength(VehiclesIDs{id});
end
if ~isempty(VehiclesIDs)
    veh1IDs=VehiclesPositions(VehiclesPositions>0);
    if isempty(veh1IDs)==0
        veh1ID=find(VehiclesPositions==min(VehiclesPositions(VehiclesPositions>0)));
        s_rampf=VehiclesPositions(veh1ID);
        v_rampf=VehiclesSpeeds(veh1ID);
        l_rampf=VehiclesLengths(veh1ID);
    else
        s_rampf=200;
        v_rampf=20;
        l_rampf=5;
    end
else
    s_rampf=200;
    v_rampf=20;
    l_rampf=5;
end
end
function [s_veh1,v_veh1,s_veh1apostrophe1,s_veh_length,s_veh_width,s_vehapostrophe_length,s_vehapostrophe_width]=AvoOncomingEnvVehInform(RoadlaneID1,RoadlaneID2,RoadlaneID3,disroad2converge)%顺对向车行驶方向123
lenRoad1=traci.lane.getLength(RoadlaneID1);
lenRoad2=traci.lane.getLength(RoadlaneID2);
oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs(RoadlaneID1) traci.lane.getLastStepVehicleIDs(RoadlaneID2) traci.lane.getLastStepVehicleIDs(RoadlaneID3)];
oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
oncomingVehiclesLengths=zeros(length(oncomingVehiclesIDs),1);
oncomingVehiclesWidths=zeros(length(oncomingVehiclesIDs),1);
for id=1:length(oncomingVehiclesIDs)
    oncomingVehiclePosition=traci.vehicle.getLanePosition(oncomingVehiclesIDs{id}); 
    oncomingVehicleRoadLaneID=traci.vehicle.getLaneID(oncomingVehiclesIDs{id});
    if strcmp(oncomingVehicleRoadLaneID,RoadlaneID1)
        oncomingVehiclesPositions(id)=lenRoad1+disroad2converge-oncomingVehiclePosition;
    elseif strcmp(oncomingVehicleRoadLaneID,RoadlaneID2)
        oncomingVehiclesPositions(id)=disroad2converge-oncomingVehiclePosition;
    elseif strcmp(oncomingVehicleRoadLaneID,RoadlaneID3)
        oncomingVehiclesPositions(id)=-(lenRoad2-disroad2converge)-oncomingVehiclePosition;
    end
    oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
    oncomingVehiclesLengths(id)=traci.vehicle.getLength(oncomingVehiclesIDs{id});
    oncomingVehiclesWidths(id)=traci.vehicle.getWidth(oncomingVehiclesIDs{id});
end
if ~isempty(oncomingVehiclesIDs)
    veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>0);
    veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<=0);
    if isempty(veh1IDs)==0
        veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>0)));
        s_veh1=oncomingVehiclesPositions(veh1ID);
        v_veh1=oncomingVehiclesSpeeds(veh1ID);
        s_veh_length=oncomingVehiclesLengths(veh1ID);
        s_veh_width= oncomingVehiclesWidths(veh1ID);
    else
        s_veh1=200;
        v_veh1=0;
        s_veh_length=5;
        s_veh_width= 1.8;
    end
    if isempty(veh1apostrophe1IDs)==0
        veh1apostrophe1ID=oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0));
        s_veh1apostrophe1=oncomingVehiclesPositions(veh1apostrophe1ID);
        s_vehapostrophe_length=oncomingVehiclesLengths(veh1apostrophe1ID);
        s_vehapostrophe_width= oncomingVehiclesWidths(veh1apostrophe1ID);
    else
        s_veh1apostrophe1=-200;
        s_vehapostrophe_length=5;
        s_vehapostrophe_width= 1.8;
    end
else
    s_veh1=200;
    v_veh1=0;
    s_veh1apostrophe1=-200;
    s_veh_length=5;
    s_veh_width= 1.8;
    s_vehapostrophe_length=5;
    s_vehapostrophe_width= 1.8;
end
end
function [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition)
% NumOfLanes=2;
% 搜寻右车
lane=max([0 lane]);
lane=min([NumOfLanes-1 lane]);
RightLaneIndex=[current_road_ID '_' num2str(round(max([0 lane-1])))];
vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(RightLaneIndex);
vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
vehicles_targetLaneLengths=zeros(length(vehicles_targetLaneIDs),1);
vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
for id=1:length(vehicles_targetLaneIDs)
    vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
    vehicles_targetLaneLengths(id)=traci.vehicle.getLength(vehicles_targetLaneIDs{id});
    vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
end
bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
if isempty(bIDs)==0
    bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
    RightLaneFrontVel=vehicles_targetLaneSpeeds(bID);
    RightLaneFrontDis=vehicles_targetLanePoses(bID);
    RightLaneFrontLen=vehicles_targetLaneLengths(bID);
else
    RightLaneFrontVel=20;
    RightLaneFrontDis=200;
    RightLaneFrontLen=5;
end
if isempty(cIDs)==0
    cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
    RightLaneBehindVel=vehicles_targetLaneSpeeds(cID);
    RightLaneBehindDis=vehicles_targetLanePoses(cID);
    RightLaneBehindLen=vehicles_targetLaneLengths(cID);
else
    RightLaneBehindVel=20;
    RightLaneBehindDis=-200;
    RightLaneBehindLen=5;
end
% 搜寻左车
LeftLaneIndex=[current_road_ID '_' num2str(round(min([NumOfLanes-1 lane+1])))];
vehicles_targetLaneIDs=traci.lane.getLastStepVehicleIDs(LeftLaneIndex);
vehicles_targetLaneSpeeds=zeros(length(vehicles_targetLaneIDs),1);
vehicles_targetLaneLengths=zeros(length(vehicles_targetLaneIDs),1);
vehicles_targetLanePoses=zeros(length(vehicles_targetLaneIDs),1);
for id=1:length(vehicles_targetLaneIDs)
    vehicles_targetLaneSpeeds(id)=traci.vehicle.getSpeed(vehicles_targetLaneIDs{id});
    vehicles_targetLaneLengths(id)=traci.vehicle.getLength(vehicles_targetLaneIDs{id});
    vehicles_targetLanePoses(id)=traci.vehicle.getLanePosition(vehicles_targetLaneIDs{id})-currentLanePosition;
end
bIDs=vehicles_targetLanePoses(vehicles_targetLanePoses>0);
cIDs=vehicles_targetLanePoses(vehicles_targetLanePoses<0);
if isempty(bIDs)==0
    bID=find(vehicles_targetLanePoses==min(vehicles_targetLanePoses(vehicles_targetLanePoses>0)));
    LeftLaneFrontVel=vehicles_targetLaneSpeeds(bID);
    LeftLaneFrontDis=vehicles_targetLanePoses(bID);
    LeftLaneFrontLen= vehicles_targetLaneLengths(bID);
else
    LeftLaneFrontVel=20;
    LeftLaneFrontDis=200;
    LeftLaneFrontLen= 5;
end
if isempty(cIDs)==0
    cID=find(vehicles_targetLanePoses==max(vehicles_targetLanePoses(vehicles_targetLanePoses<0)));
    LeftLaneBehindVel=vehicles_targetLaneSpeeds(cID);
    LeftLaneBehindDis=vehicles_targetLanePoses(cID);
    LeftLaneBehindLen=vehicles_targetLaneLengths(cID);
else
    LeftLaneBehindVel=20;
    LeftLaneBehindDis=-200;
    LeftLaneBehindLen=5;
end
end
function [pos_s,s_ped,l_ped,v_ped,psi_ped]=...
    AvoPedInform(RoadlaneID1,PedRoad1,PedRoad2,PedRoad3,pos_s)
s_ped=zeros(1,40,'double');
l_ped=zeros(1,40,'double');
v_ped=zeros(1,40,'double')-1;
psi_ped=zeros(1,40,'double');
% laneshape=traci.lane.getShape('E25_5');
laneshape=traci.lane.getShape(RoadlaneID1);
PersonIDs=traci.edge.getLastStepPersonIDs(PedRoad1);
if ~isempty(PedRoad2)&&~isempty(PedRoad3)
    PersonID2s=traci.edge.getLastStepPersonIDs(PedRoad2);
    PersonID3s=traci.edge.getLastStepPersonIDs(PedRoad3);
    PersonIDs=[PersonIDs PersonID2s PersonID3s];
elseif ~isempty(PedRoad2)
    PersonID2s=traci.edge.getLastStepPersonIDs(PedRoad2);
    PersonIDs=[PersonIDs PersonID2s];
elseif ~isempty(PedRoad3)
    PersonID3s=traci.edge.getLastStepPersonIDs(PedRoad3);
    PersonIDs=[PersonIDs PersonID3s];
end
personPositions=zeros(length(PersonIDs),4);
for id=1:length(PersonIDs)
    personPosition=traci.person.getPosition(PersonIDs{id});
    personPositions(id,1)=personPosition(2);
    personPositions(id,2)=laneshape{1,1}(1,1)-personPosition(1);
    personPositions(id,3)=traci.person.getSpeed(PersonIDs{id});
    personPositions(id,4)=90+traci.person.getAngle(PersonIDs{id});
end
personPositions=sortrows(personPositions,1);
if ~isempty(PersonIDs)
    s_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,1)';
    l_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,2)';
    v_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,3)';
    psi_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,4)';
end
end
% function [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform2(RoadlaneID1,RoadlaneID2,RoadlaneID3,PedRoad1,pos_s,current_lane_ID)%左转
% % lenRoad1=traci.lane.getLength(RoadlaneID1);
% lenRoad2=traci.lane.getLength(RoadlaneID2);
% lenRoad3=traci.lane.getLength(RoadlaneID3);
% laneshape2=traci.lane.getShape(RoadlaneID2);
% laneshape3=traci.lane.getShape(RoadlaneID3);
% s_end_cross=laneshape2{1,1}(1,2)+lenRoad2+lenRoad3;
% 
% s_ped=zeros(1,40,'double');
% l_ped=zeros(1,40,'double');
% v_ped=zeros(1,40,'double')-1;
% psi_ped=zeros(1,40,'double');
% % laneshape=traci.lane.getShape('E25_5');
% PersonIDs=traci.edge.getLastStepPersonIDs(PedRoad1);
% personPositions=zeros(length(PersonIDs),4);
% for id=1:length(PersonIDs)
%     personPosition=traci.person.getPosition(PersonIDs{id});
%     personPositions(id,1)=s_end_cross-abs(personPosition(1)-laneshape3{1,end}(1,1));
%     personPositions(id,2)=laneshape3{1,end}(1,2)-personPosition(2);
%     personPositions(id,3)=traci.person.getSpeed(PersonIDs{id});
%     personPositions(id,4)=traci.person.getAngle(PersonIDs{id})-180;
% end
% personPositions=sortrows(personPositions,1);
% if ~isempty(PersonIDs)
%     s_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,1)';
%     l_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,2)';
%     v_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,3)';
%     psi_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,4)';
% end
% if strcmp(current_lane_ID,RoadlaneID1)
%     pos_s=pos_s;
% elseif strcmp(current_lane_ID,RoadlaneID2)
%     pos_s=traci.vehicle.getLanePosition('S0')+laneshape2{1,1}(1,2);
% elseif strcmp(current_lane_ID,RoadlaneID3)
%     pos_s=traci.vehicle.getLanePosition('S0')+laneshape2{1,1}(1,2)+lenRoad2;
% end
% end
function [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3(RoadlaneID1,RoadlaneID2,RoadlaneID3,PedRoad1,pos_s,current_lane_ID,turn)%右转&左转

% lenRoad1=traci.lane.getLength(RoadlaneID1);
lenRoad2=traci.lane.getLength(RoadlaneID2);
laneshape2=traci.lane.getShape(RoadlaneID2);
if ~isempty(RoadlaneID3)  
lenRoad3=traci.lane.getLength(RoadlaneID3);
laneshape3=traci.lane.getShape(RoadlaneID3);
else
    lenRoad3=0;
    laneshape3=laneshape2;
end

s_end_cross=laneshape2{1,1}(1,2)+lenRoad2+lenRoad3;

s_ped=zeros(1,40,'double');
l_ped=zeros(1,40,'double');
v_ped=zeros(1,40,'double')-1;
psi_ped=zeros(1,40,'double');
% laneshape=traci.lane.getShape('E25_5');
PersonIDs=traci.edge.getLastStepPersonIDs(PedRoad1);
personPositions=zeros(length(PersonIDs),4);
for id=1:length(PersonIDs)
    personPosition=traci.person.getPosition(PersonIDs{id});
    personPositions(id,1)=s_end_cross-abs(personPosition(1)-laneshape3{1,end}(1,1));
    if strcmp(turn,'turnright')
        personPositions(id,2)=personPosition(2)-laneshape3{1,end}(1,2);
        personPositions(id,4)=traci.person.getAngle(PersonIDs{id});
    elseif strcmp(turn,'turnleft')
        personPositions(id,2)=laneshape3{1,end}(1,2)-personPosition(2);
        personPositions(id,4)=traci.person.getAngle(PersonIDs{id})-180;
    end
    personPositions(id,3)=traci.person.getSpeed(PersonIDs{id});
    
end
personPositions=sortrows(personPositions,1);
if ~isempty(PersonIDs)
    s_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,1)';
    l_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,2)';
    v_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,3)';
    psi_ped(1,1:min(40,length(personPositions(:,1))))=personPositions(:,4)';
end
if strcmp(current_lane_ID,RoadlaneID1)
%     pos_s=pos_s;
elseif strcmp(current_lane_ID,RoadlaneID2)
    pos_s=traci.vehicle.getLanePosition('S0')+laneshape2{1,1}(1,2);
elseif strcmp(current_lane_ID,RoadlaneID3)
    pos_s=traci.vehicle.getLanePosition('S0')+laneshape2{1,1}(1,2)+lenRoad2;
end
end

