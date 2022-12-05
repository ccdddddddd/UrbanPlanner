clearvars -except pathcase pathID PlannerLevel SampleTime;
close('all');
clc;
 %入参
 if exist('PlannerLevel','var')==0
     PlannerLevel=int16(1);%1、轨迹规划，2、车辆行为决策  3、车辆行为提示驾驶员
 end
 if exist('SampleTime','var')==0
     SampleTime=0.1;
 end
 manual=0;%0,无图，1、按键，2、画图所有决策结果、3、画图行为决策结果文字输出、4、画图控制决策文字输出
 frenetflag=1;%0原frenet坐标系，1新优化frenet坐标系
 GlosaActive=int16(0);
%% 写入City.sumocfg仿真步长
if exist('pathID','var')==1
    sumocfg=xmlread('City.sumocfg');
    sumocfgtime= sumocfg.getElementsByTagName('step-length');
    SampleTimeOld=str2double(sumocfgtime.item(0).getAttribute('value'));
    if SampleTimeOld~=SampleTime
        sumocfgtime.item(0).setAttribute('value',num2str(SampleTime));
        xmlwrite([pathID '\' 'City.sumocfg'],sumocfg);
    end
end
% type('City.sumocfg');
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
    
    itemIndex=find(strcmp(tlLogic(:,1),{'J6'}),1);
    if ~isempty(itemIndex)
        tlLogicItem=tlLogicItems.item(itemIndex-1);%找元素tlLogic，中属性id是'J6'
        phaseItems = tlLogicItem.getElementsByTagName('phase');
        NumofElements=phaseItems.getLength;
        if NumofElements~=0
            phase{NumofElements,2}=[];
            for item=1:NumofElements
                phase(item,1)=phaseItems.item(item-1).getAttribute('duration');
                phase(item,2)=phaseItems.item(item-1).getAttribute('state');
            end
        end
        %信号灯全局变量------------------------------------------------------------------------
        TL6phase0=str2double(phase(1,1));
        TL6phase1=str2double(phase(2,1));
        TL6phase2=str2double(phase(3,1));
        TL6phase3=str2double(phase(4,1));
        %------------------------------------------------------------------------
    end
end
%% 初始化全局变量
% 设置参数
import traci.constants;
v_max=50/3.6;
NumofLane=1;
%基本参数
Parameters.turningRadius=double(TurningRadius);%20220323
Parameters.w_veh=double(1.8);
Parameters.l_veh=double(5);
%标定量
CalibrationVars.TrajPlanTurnAround.d_safe1=0.5;
CalibrationVars.TrajPlanTurnAround.d_safe2=2;
CalibrationVars.TrajPlanTurnAround.dec2line=0.2;
CalibrationVars.TrajPlanTurnAround.a_min=-2;
CalibrationVars.TrajPlanTurnAround.a_max_com=1.5;
CalibrationVars.TrajPlanTurnAround.v_max_turnAround=5;
CalibrationVars.TrajPlanTurnAround.d_gap2stop=0.1;
CalibrationVars.SpeedPlanAvoidPedestrian.a_max=2.5;
CalibrationVars.SpeedPlanAvoidPedestrian.a_min=-2;
CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int=30/3.6;
CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg=20/3.6;
CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped=0.5;
CalibrationVars.SpeedPlanTrafficLight.a_min_com=-1.5;
CalibrationVars.SpeedPlanTrafficLight.a_max=2.5;
CalibrationVars.SpeedPlanTrafficLight.a_min=-3;
CalibrationVars.SpeedPlanTrafficLight.v_max_int=30/3.6;
CalibrationVars.SpeedPlanTrafficLight.t_acc=1.5;
CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline=0.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_min_com=-1.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_max=1.5;
CalibrationVars.SpeedPlanAvoidVehicle.a_min=-3;
CalibrationVars.SpeedPlanAvoidVehicle.v_max=40/3.6;
CalibrationVars.SpeedPlanAvoidVehicle.t_re=1.5;
CalibrationVars.SpeedPlanAvoidVehicle.gapIndex=2;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com=1.5;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min=-3;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int=30/3.6;
CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe=2;
CalibrationVars.TrajPlanLaneChange.v_max_int=30/3.6;
CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int=1;
CalibrationVars.TrajPlanLaneChange.t_permit=3;
CalibrationVars.TrajPlanLaneChange.t_re=0.5;
CalibrationVars.TrajPlanLaneChange.index_accel=0.5;
CalibrationVars.TrajPlanLaneChange.a_max_comfort=1;
CalibrationVars.TrajPlanLaneChange.a_min=-3.5;
CalibrationVars.TrajPlanLaneChange.a_max=2.5;
CalibrationVars.TrajPlanLaneChange.a_min_comfort=-1;
CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit=15; % 初始值15度
CalibrationVars.TrajPlanLaneChange.a_lateral=4; % 默认为4
CalibrationVars.ACC.a_max=2.5;
CalibrationVars.ACC.a_min=-4;
CalibrationVars.ACC.d_wait2faultyCar=13;
CalibrationVars.ACC.tau_v_com=4;
CalibrationVars.ACC.tau_v=2;
CalibrationVars.ACC.tau_d=5;
CalibrationVars.ACC.tau_v_bre=1;
CalibrationVars.ACC.tau_v_emg=0.5;
CalibrationVars.ACC.tau_d_emg=2;
CalibrationVars.ACC.t_acc=2;
CalibrationVars.ACC.d_wait=4;
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
CalibrationVars.ACClowSpeed.d_wait=4;%4
CalibrationVars.Decider.a_bre=-3;%m/s^2
CalibrationVars.Decider.a_bre_com=-1.5;%m/s^2
CalibrationVars.Decider.idle_speed=7;%km/h
CalibrationVars.Decider.dist_wait2pilot=10;%m
CalibrationVars.Decider.dist_wait2veh=15;%m
CalibrationVars.Decider.glosaAdp=1.5;
CalibrationVars.Decider.mrg=4;
CalibrationVars.Decider.desRate=0.75;
CalibrationVars.Decider.dIntxn=10;
CalibrationVars.Decider.dMin=2;
CalibrationVars.Decider.dec=1;
CalibrationVars.Decider.glosaAverageIndex=0.8;
% 全局变量
GlobVars.AEBDecision.AEBActive=int16(0);
GlobVars.TrajPlanTurnAround.posCircle=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.posCircle2=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.posCircle3=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.pos_start=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.pos_mid1=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid2=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid1_rear=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_mid2_rear=zeros(1,4,'double');
GlobVars.TrajPlanTurnAround.pos_end=zeros(1,2,'double');
GlobVars.TrajPlanTurnAround.laneCenterline=zeros(1,7,'double');
GlobVars.TrajPlanTurnAround.dec_trunAround=int16(0);
GlobVars.TrajPlanTurnAround.wait_turnAround=int16(0);
GlobVars.TrajPlanTurnAround.typeOfTurnAround=int16(0);
GlobVars.TrajPlanTurnAround.turnAroundState=int16(0);
GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite=int16(0);
GlobVars.TrajPlanTurnAround.turnAroundActive=int16(0);
GlobVars.TrajPlanTurnAround.reflineSend=0;
GlobVars.TrajPlanTurnAround.reflineLend=0;
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
GlobVars.TrajPlanLaneChange.countLaneChange=int16(0);
GlobVars.TrajPlanLaneChange.durationLaneChange=int16(0);
GlobVars.TrajPlanLaneChange.laneChangePath=zeros([6/0.05 6],'double');
GlobVars.TrajPlanLaneChange.t_lc_traj=double(2);
GlobVars.TrajPlanLaneChange.currentTargetLaneIndex=int16(0);
GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan=int16(0);
GlobVars.TrajPlanLaneChange_RePlan.para=[0 0 0 0 0 0]; % 初始值[0 0 0 0 0 0]
GlobVars.TrajPlanLaneChange_RePlan.s_end=0; % 初始值0
GlobVars.TrajPlanLaneChange_RePlan.l_end=0; % 初始值0
GlobVars.TrajPlanLaneChange_RePlan.para1=zeros(1,5,'double');
GlobVars.TrajPlanLaneChange_RePlan.para2=zeros(4,4,'double');
GlobVars.TrajPlanLaneChange_RePlan.para3=0;
GlobVars.Decider.dec_start=int16(0);
GlobVars.Decider.dir_start=int16(0);
GlobVars.Decider.countLaneChangeDecider=int16(0);
GlobVars.Decider.currentTargetLaneIndexDecider=int16(0);
GlobVars.Decider.a_soll_pre=100;
GlobVars.Decider.a_sollpre2traj=100;
GlobVars.Decider.wait_pullover=int16(0);
GlobVars.Decider.distBehindGoal=0;
GlobVars.Decider.dec_follow=int16(0);
GlobVars.SpeedPlanStopSign.wait_stopsign=int16(0);
Decision.TargetGear=int16(0);
CurrentGear=0;
RefLaneIndex=0;
Reflane_ID=0;
Reflane_ID_pre='0';
nodelist_s=zeros(1,6,'double')+1000;%一个lan折最大5段
nodelist_s(1)=0;
TimeResponse=0;
TimeResponse1=0;
TimeResponse2=0;
AEBdelay=0;
StopVeh=0;
Fllowsts=0;
global_route={};
turnAroundMoveToflag=0;
current_road_ID_pre=0;%前一帧linkID
WidthOfLanes=[0,0,0,0,0,0];%当前路车道宽度（全局变量）
TrafficLightActive=int16(0);
TrafficLightActivePre=int16(0);
tlsID='0';
TurnAroundReflineState=0;%决策掉头参考线下发状态
 %-------------------------------------------------------------------
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
    path=strcat('Sumocfg/D20_Decider_LaneChange/Cityplanner',num2str(usecase),'/');
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
    path=strcat('Sumocfg/D21_Decider_SlowDown/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif (usecase>=218&&usecase<=231)||(usecase>=236&&usecase<=241)
    path=strcat('Sumocfg/D22_Decider_Wait_Start/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=132&&usecase<=144
    path=strcat('Sumocfg/17_PullOver/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=145&&usecase<=151
    path=strcat('Sumocfg/18_StopSign/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=174&&usecase<=178
    path=strcat('Sumocfg/19_Event/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=232&&usecase<=235
    path=strcat('Sumocfg/D23_Decider_AEB/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=179&&usecase<=185
    path=strcat('Sumocfg/20_DeviationCompensate/Cityplanner',num2str(usecase),'/');
    traci.start(strcat('sumo-gui -c ./',path,'City.sumocfg --start'));
elseif usecase>=242&&usecase<=247
    path=strcat('Sumocfg/D24_Glosa/Cityplanner',num2str(usecase),'/');
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
%% 画图
if manual==2||(usecase>=201&&usecase<=205)
    t=linspace(0,(duration-1)/10,duration/(SampleTime*10));
    w=zeros(1,duration/(SampleTime*10))*nan;
%     w(:,700)=[0];
    w(1,700/(SampleTime*10))=0;
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
%     LC_text=text(x+22,2.5,['LC' 32 '=' 32 num2str(0)]);
%     T1=text(x+1,2.8,'纵向指令');
%     T2=text(x+12,2.8,'控制量' );
%     T3=text(x+30,2.8,'横向指令');
%     T4=text(x+40,2.8,'接管指令');
    Title=title(['TargetLane=' num2str(0) 32 32 32 32 'CurrentLane=' num2str(0)]);
    xlabel('t')
    yticklabels({'Keep','Left','Right'});
    yticks(0:0.5:1);
    drawnow
    %------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
elseif manual==4
    t=linspace(0,(duration-1)/10,duration/(SampleTime*10));
    w=zeros(1,duration/(SampleTime*10))*nan;
%     w(:,700)=[0];
    w(1,700/(SampleTime*10))=0;
    p=plot(t,w,'.-');
    x=45;
    axis([x x+50 -0.5 3]);
    grid on;
%     label=legend('Decision.LaneChange');
    set(gcf,'Position',[1+160,40+200,540,290])
%     set(gcf,'Position',[1,40,540,290])
    get(gcf,'Position');
     g=text(x+12,2.5,['speed' 32 '=' 32 num2str(0) 'km/h']);
    wait_text=text(x+1,2.7,['wait' 32 '=' 32 num2str(0)]);
    waitDist_text=text(x+1,2.5,['dist' 32 32 '=' 32 num2str(0) 'm']);
    slowdown_text=text(x+12,2.9,['slowd' 32 32 '=' 32 num2str(0)]);
    targetspeed_text=text(x+12,2.7,['targV' 32 32 '=' 32 num2str(0) 'km/h']);
%     start_text=text(x+1,2.9,['start' 32 '=' 32 num2str(0)]);
    AEB_text=text(x+22,2.5,['AEB' 32 '=' 32 num2str(0)]);
    LC_text=text(x+22,2.5,['LC' 32 '=' 32 num2str(0)]);
    T1=text(x+1,2.8,'纵向指令');
    T2=text(x+12,2.8,'控制量' );
    T3=text(x+30,2.8,'横向指令');
    T4=text(x+40,2.8,'接管指令');
%     Title=title(['TargetLane=' num2str(0) 32 32 32 32 'CurrentLane=' num2str(0)]);
    xlabel('t')
    yticklabels({'Keep','Left','Right'});
    yticks(0:0.5:1);
    drawnow 
    %-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
%%
delay=0;
%% sumo-gui画多边形
% function add(polygonID, shape, color, fill, polygonType, layer, lineWidth)
traci.polygon.add('polygonID', {[-100 55] [-100 89]}, [255 255 255 255], false,'1',4,0.05);%人行道导致无车道边线，增加白线
% traci.polygon.add('polygonID1', {[-100 86] [-100 89]}, [255 255 255 255], false,'1',4,0.05);
traci.polygon.add('polygonID2', {[112.8 -170] [116 -170]}, [255 0 0 255], false,'1',4,0.4);%靠边停车停止线(远)
for traj_index=1:80%轨迹点
    traci.polygon.add(['traj' num2str(traj_index)], {[0 0] [0+0.01 0+0.01]}, [230 255 0 255], false,'1',6,0.08);
end
if usecase==137||usecase==140||usecase==141||usecase==238%靠边停车停止线（近）
    traci.polygon.add('polygonID4', {[112.8 -210] [116 -210]}, [255 0 0 255], false,'1',4,0.4);
else
    traci.polygon.add('polygonID4', {[112.8 -210] [116 -210]}, [255 255 255 255], false,'1',4,0.05);
end
if usecase==100||usecase==206%靠边停车停止线
    traci.polygon.add('polygonID5', {[-250 320] [-250 323.2]}, [255 0 0 255], false,'1',4,0.4);
end
%% 仿真退出逻辑
for i = 1:SampleTime*10: duration
    traci.simulation.step();
    if i>700
           usecase
        if GlobVars.AEBDecision.AEBActive>0
            delay=delay+1;
        elseif exist('d_veh2goal','var')==1&&d_veh2goal<0.25&&speed==0
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
%         TrafficLightActive=int16(0);
        VehicleCrossingActive=int16(0);
        VehicleOncomingActive=int16(0);
%         WidthOfLanes=[0,0,0,0,0,0];
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
        AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle=zeros(1,4,'double')-200;
        AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle=zeros(1,4,'double')+20;
        AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle=zeros(1,4,'double')+200;
        AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle=zeros(1,4,'double')+20;
        AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle=zeros(1,4,'double')+5;
        AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle=zeros(1,4,'double')+5;
        d_veh2int=200;
        d_veh2converge=200;
        d_veh2stopline=200;
        s_ped=zeros(1,40,'double');
        l_ped=zeros(1,40,'double');
        v_ped=zeros(1,40,'double')-1;
        psi_ped=zeros(1,40,'double');
        d_veh2cross=200;
        greenLight=1;
%         time2nextSwitch=100;
        trafficLightPhase=zeros(1,10);
        LanesWithFail=zeros(1,6,'int16');
        FailLaneindex=zeros(1,5,'int16');
        FailLaneFrontDis=zeros(1,5,'double');
        FailLaneFrontVel=zeros(1,5,'double');
        FailLaneFrontLen=zeros(1,5,'double')+5;
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
        NumOfLanesOpposite=0;
        WidthOfLanesOpposite=[3.2,3.2,3.2,0,0,0];
        WidthOfGap=0;
        s_turnaround_border=0;
        IndexOfLaneOppositeCar=zeros([20,1],'int16');
        SpeedOppositeCar=zeros([20,1],'double');
        PosSOppositeCar=zeros([20,1],'double');
        PosSOppositeCar=PosSOppositeCar-200;
        LengthOppositeCar=zeros([20,1],'double')+5;
        IndexOfLaneCodirectCar=zeros([10,1],'int16');
        SpeedCodirectCar=zeros([10,1],'double')-1;
        PosSCodirectCar=zeros([10,1],'double');  
        LengthCodirectCar=zeros([10,1],'double')+5;
        traci.vehicle.setSpeedMode('S0', 0)
        % set view of sumo gui
        if i==701
            traci.gui.trackVehicle('View #0','S0');
            if usecase==101||usecase==102||usecase==103||usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241)
                traci.gui.setZoom('View #0', 600);
            elseif (usecase>=174&&usecase<=178)
                traci.gui.setZoom('View #0', 900);
            elseif usecase==110||usecase==111
                traci.gui.setZoom('View #0', 1800);
            elseif (usecase>=126&&usecase<=131)||(usecase>=112&&usecase<=114)
                traci.gui.setZoom('View #0', 2000);
            elseif (usecase>=132&&usecase<=144)||usecase==147||(usecase>=232&&usecase<=235)
                traci.gui.setZoom('View #0', 1500);
            elseif (usecase>=246&&usecase<=247)
                traci.gui.setZoom('View #0', 800);
            else
                traci.gui.setZoom('View #0', 1200);
            end
        end
        %traci.gui.setBoundary('View #0',135,85,160,110);
        yaw=traci.vehicle.getAngle('S0');
        lane=traci.vehicle.getLaneIndex('S0') ;
        speed=max([0 traci.vehicle.getSpeed('S0')]) ;
        current_road_ID=traci.vehicle.getRoadID('S0');
        current_lane_ID=traci.vehicle.getLaneID('S0');
        postion_veh=traci.vehicle.getPosition('S0');
%         suvscri=traci.vehicle.getSubscriptionResults('S0');
        if i>=720&&usecase~=100
             route=global_route; 
        else 
        route = traci.vehicle.getRoute('S0');%避免每一帧都get
        end
        if isempty(global_route)
            global_route= route;
        elseif length(global_route)~=length(route)
            if any(strcmp(current_road_ID,global_route))
                global_route=global_route(1,find(strcmp(current_road_ID,global_route)):end);
                traci.vehicle.setRoute('S0',global_route);
            end
        end
        currentLanePosition=traci.vehicle.getLanePosition('S0');
        NumofLane=traci.edge.getLaneNumber(current_road_ID);
%         RouteIndex=traci.vehicle.getRouteIndex('S0');
%         RouteID=traci.vehicle.getRouteID('S0');
%         routeIDlist=traci.route.getIDList;
%         RoutingMode=traci.vehicle.getRoutingMode('S0');
%         getDistance=traci.vehicle.getDistance('S0');
% %         getDrivingDistance=traci.vehicle.getDrivingDistance('S0');
%         getLanePosition=traci.vehicle.getLanePosition('S0');
%         getLateralLanePosition=traci.vehicle.getLateralLanePosition('S0');
%         getLinks= traci.lane.getLinks(current_lane_ID);
%         NextTLS=traci.vehicle.getNextTLS('S0') ;
%         a=traci.lane.getLinkNumber(current_lane_ID);
%         b=traci.lane.getLinks(current_lane_ID);
      %% 计算当前道路车道宽
      if current_road_ID_pre==0%第一帧或者道路ID变化时计算当前道路车道宽度
          for WidthIndex=1:NumofLane 
        %           WidthOfLanes(WidthIndex)=3.2;
             WidthOfLanes(WidthIndex) = traci.lane.getWidth([current_road_ID '_' num2str(NumofLane-WidthIndex)]); 
          end
      elseif strcmp(current_road_ID_pre,current_road_ID)==0
          for WidthIndex=1:NumofLane 
             WidthOfLanes(WidthIndex) = traci.lane.getWidth([current_road_ID '_' num2str(NumofLane-WidthIndex)]); 
          end
      end
      current_road_ID_pre=current_road_ID;%更新全局变量
      %% 当前车道与目标车道
        CurrentLaneIndex=NumofLane-lane;
        if CurrentLaneIndex>6
            CurrentLaneIndex=1;
        end
        if strcmp(current_road_ID,'E15')&&currentLanePosition>25
            TargetLaneIndex=2;
        elseif strcmp(current_road_ID,'E12')&&(usecase<=88||usecase>=156)
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
        elseif strcmp(current_road_ID,'E25')&&(usecase==100||usecase==206)
            TargetLaneIndex=2;
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
        if strcmp(current_road_ID,'E25')&&strcmp(route{length(route)-1},'E25')
            if usecase>=42&&usecase<=44
                [aID, adist]=traci.vehicle.getLeader('S0',100);
                if isempty(aID)==0
                    FrontDis=adist+traci.vehicle.getMinGap('S0')+traci.vehicle.getLength(aID);
                else
                    FrontDis=200;
                end
            else 
                FrontDis=200;
            end
            if FrontDis<=25&&usecase==42
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif FrontDis<=20&&usecase==43
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif FrontDis<=15&&usecase==44
                TargetLaneIndex=CurrentLaneIndex+1;
            elseif usecase==45|| usecase==46||usecase==61||usecase==62||usecase==63
                TargetLaneIndex=4;
            elseif usecase>=132&&usecase<=143
                TargetLaneIndex=6;
            elseif usecase==144
                TargetLaneIndex=5;
            end
        end
        if strcmp(current_road_ID,'E12')&&usecase==185
            TargetLaneIndex=2;
        end
        %% 参考线更新
%         if RefLaneIndex==0
%             RefLaneIndex=CurrentLaneIndex;
%         elseif (GlobVars.TrajPlanLaneChange.durationLaneChange==0 || GlobVars.TrajPlanLaneChange.durationLaneChange>GlobVars.TrajPlanLaneChange. t_lc_traj/0.1) ...
%                 && (GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0 || GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan> GlobVars.TrajPlanLaneChange_RePlan.t_lc_RePlan/0.1) ...,
%                 && (GlobVars.TrajPlanTurnAround.turnAroundActive==0)
%             RefLaneIndex=CurrentLaneIndex;
%         end
%         if Reflane_ID==0
%             Reflane_ID=current_lane_ID;
%         elseif  (GlobVars.TrajPlanLaneChange.durationLaneChange==0 || GlobVars.TrajPlanLaneChange.durationLaneChange>GlobVars.TrajPlanLaneChange. t_lc_traj/0.1) ...
%                 && (GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0 || GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan> GlobVars.TrajPlanLaneChange_RePlan.t_lc_RePlan/0.1) ...,
%                 && (GlobVars.TrajPlanTurnAround.turnAroundActive==0)
%             Reflane_ID=current_lane_ID;
%         end
        if RefLaneIndex==0
            RefLaneIndex=CurrentLaneIndex;
        elseif (GlobVars.TrajPlanLaneChange.durationLaneChange==0 || GlobVars.TrajPlanLaneChange.durationLaneChange>GlobVars.TrajPlanLaneChange. t_lc_traj/0.1) ...
                && (GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0 ) ...,
                && TurnAroundReflineState==0
            RefLaneIndex=CurrentLaneIndex;
        end
        if Reflane_ID==0
            Reflane_ID=current_lane_ID;
        elseif  (GlobVars.TrajPlanLaneChange.durationLaneChange==0 || GlobVars.TrajPlanLaneChange.durationLaneChange>GlobVars.TrajPlanLaneChange. t_lc_traj/0.1) ...
                && (GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0 ) ...,
                && TurnAroundReflineState==0
            Reflane_ID=current_lane_ID;
        end
        %% 本车frenet坐标
        if frenetflag==1
            laneshape=traci.lane.getShape(Reflane_ID);
            if strcmp(Reflane_ID_pre,Reflane_ID)==0%参考线被更新了
                for node_index=2:length(laneshape)
                    snode_x=laneshape{1,node_index-1}(1,1);
                    snode_y=laneshape{1,node_index-1}(1,2);
                    enode_x=laneshape{1,node_index}(1,1);
                    enode_y=laneshape{1,node_index}(1,2);
                    theta=atan2((enode_y-snode_y),(enode_x-snode_x))*180/pi;
                    nodelist_s(node_index)=(enode_x-snode_x)*cosd(theta)+(enode_y-snode_y)*sind(theta);
                    nodelist_s(node_index)=nodelist_s(node_index)+nodelist_s(node_index-1);
                end
            end
            Reflane_ID_pre=Reflane_ID;%更新参考线全局变量
            [pos_s,pos_l,pos_psi]=XY2frenet(postion_veh(1),postion_veh(2),yaw,nodelist_s,laneshape);
            if length(laneshape)>2||pos_s<=4
                pos_psi=90;
            end
            pos_l_CurrentLane=0;
            
        else
            if strcmp(current_road_ID,'E25')
                pos_l_CurrentLane=0;
                pos_s=postion_veh(2);
                pos_psi=mod(yaw+90,360);
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
            elseif strcmp(current_road_ID,'E12')
                pos_l_CurrentLane=0;
                pos_s=postion_veh(2);
                pos_psi=mod(yaw+90,360);
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
                pos_psi=mod(yaw+90,360);
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
                pos_psi=mod(yaw+90,360);
                if pos_l>3.2
                    pos_l_CurrentLane=4.8;
                elseif pos_l>0
                    pos_l_CurrentLane=1.6;
                end
            elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
                pos_l_CurrentLane=0;
                pos_s=postion_veh(1);
                pos_psi=mod(yaw,360);
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
            elseif strcmp(current_road_ID,'8')%丁字路口前直行路
                pos_l_CurrentLane=0;
                pos_s=-postion_veh(1);
                pos_psi=mod(yaw+180,360);
                if RefLaneIndex==1
                    laneshape=traci.lane.getShape([current_road_ID '_1']);
                    pos_l=laneshape{1,1}(1,2)-postion_veh(2);
                elseif RefLaneIndex==2
                    laneshape=traci.lane.getShape([current_road_ID '_0']);
                    pos_l=laneshape{1,1}(1,2)-postion_veh(2);
                end
            elseif strcmp(current_road_ID,'-E14')%Y字路口左转后直行路（靠边停车）
                pos_l_CurrentLane=0;
                pos_s=-postion_veh(1);
                pos_psi=mod(yaw+180,360);
                if RefLaneIndex==1
                    laneshape=traci.lane.getShape([current_road_ID '_0']);
                    pos_l=laneshape{1,1}(1,2)-postion_veh(2);
                end
            elseif (strcmp(current_road_ID,'5')||strcmp(current_road_ID,'9')||strcmp(current_road_ID,'-gneE0'))&&TurnAroundReflineState==0%掉头后参考线坐标系
                pos_l_CurrentLane=0;
                pos_s=-postion_veh(2);
                pos_psi=mod(yaw-90,360);
                laneshape=traci.lane.getShape([current_road_ID '_' num2str(NumofLane-RefLaneIndex)]);
                pos_l=postion_veh(1)-laneshape{1,1}(1,1);
            elseif TurnAroundReflineState==1 %掉头参考线坐标系
                pos_s=postion_veh(2);
                pos_l=101.6-postion_veh(1);
                pos_psi=mod(yaw+90,360);
                pos_l_CurrentLane=0;
            else%保证pos_l与pos_l_CurrentLane差小于0.3
                pos_s=postion_veh(2);
                pos_l=0;
                pos_l_CurrentLane=0;
                pos_psi=90;
            end
        end
        TurnAroundMoveTo=GlobVars.TrajPlanTurnAround.turnAroundActive;%仿真时掉头轨迹标志位
        %% 终点信息
        if strcmp(current_road_ID,'-E14')%Y字路口左转后直行路（靠边停车）
            if usecase==100||usecase==206
                GoalLaneIndex=int16(1);
                d_veh2goal=postion_veh(1)+250;
            end
        elseif strcmp(current_road_ID,'E25')%（靠边停车）
            if (usecase>=132&&usecase<=144)||(usecase>=238&&usecase<=238)
                GoalLaneIndex=int16(6);
                d_veh2goal=-170-postion_veh(2);
                if usecase==137||usecase==140||usecase==141||usecase==238
                    d_veh2goal=d_veh2goal-40;
                end
            end
        end
        %% 搜寻前车
        getLeader=1;
        if (usecase>=170&&usecase<=175)||usecase==59||(usecase>=70&&usecase<=75)||usecase==224||usecase==226||usecase==227||usecase==228||usecase==234||usecase==214||usecase==154||usecase==155||(usecase==206&&strcmp(current_road_ID,'8')==0)||(usecase==206&&strcmp(current_road_ID,':J12_4')==0)
            getLeader=0;
        end
        if getLeader==1
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
        %% 搜寻不可通行车道
        if strcmp(current_road_ID,'E25')&&currentLanePosition>60&&(usecase<95||usecase==100||usecase>=206)
            for j=1:1:NumofLane
                SearchedLane=['E25_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&usecase==95
            for j=1:1:NumofLane
                SearchedLane=['E25_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>60
                LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&(usecase==96||usecase==97)
            for j=1:1:NumofLane
                SearchedLane=['E25_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>150
                LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E12')&&usecase<156
            for j=1:1:NumofLane
                SearchedLane=['E12_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'E10')
            for j=1:1:NumofLane
                SearchedLane=['E10_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                LanesWithFail=zeros(1,6,'int16');
            end
        elseif strcmp(current_road_ID,'A1')
            for j=1:1:NumofLane
                SearchedLane=['A1_' num2str(round(NumofLane-j))];
                if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1
                    LanesWithFail(j)=j;
                    LanesWithFail=sort(LanesWithFail,'descend');
                end
            end
            if currentLanePosition>100
                LanesWithFail=zeros(1,6,'int16');
            end
        end
        
        %% 事件输入
        if usecase>=174&&usecase<=178
            %异常停车、、
            if strcmp(current_road_ID,'E12')
                for j=1:1:NumofLane
                    SearchedLane=['E12_' num2str(round(NumofLane-j))];
                    if traci.lane.getLastStepHaltingNumber(SearchedLane)>0 && j~=1&&j~=3
                        LanesWithFail(j)=j;
                        LanesWithFail=sort(LanesWithFail,'descend');
                    end
                end
                if usecase==175&&currentLanePosition>70
                    LanesWithFail(1)=2;
                end  
                if currentLanePosition>120
                    LanesWithFail=zeros(1,6,'int16');
                end
            end
            %异常低速
            if usecase==175
                [aID, adist]=traci.vehicle.getLeader('S0',200);
                if isempty(aID)==0
                    L_veh1=traci.vehicle.getLength(aID);
                    Vf1=traci.vehicle.getSpeed(aID);
                    Sd1=adist+traci.vehicle.getMinGap('S0')+L_veh1;
                    if Vf1<=3&&Sd1<=20
                        LanesWithFail(1)=CurrentLaneIndex;
                    end
                end
            end
            %逆行车辆
            if usecase==176
                [aID, adist]=traci.vehicle.getLeader('S0',200);
                if isempty(aID)==0
                    L_veh1=traci.vehicle.getLength(aID);
                    Vf1=traci.vehicle.getSpeed(aID);
                    Sd1=adist+traci.vehicle.getMinGap('S0')+L_veh1;
                    if Sd1<=30
                        LanesWithFail(1)=CurrentLaneIndex;
                    end
                end
            end
            %超视距前碰撞预警
            if usecase==177
                [aID, adist]=traci.vehicle.getLeader('S0',200);
                if isempty(aID)==0
                    [aID2, adist2]=traci.vehicle.getLeader(aID,200);
                    L_veh1=traci.vehicle.getLength(aID);
                    Vf1=traci.vehicle.getSpeed(aID);
                    Sd1=adist+traci.vehicle.getMinGap('S0')+L_veh1;
                    Af1=traci.vehicle.getAcceleration(aID);
                    if isempty(aID2)==0
                        L_veh2=traci.vehicle.getLength(aID2);
                        Vf2=traci.vehicle.getSpeed(aID2);
                        Sd2=adist2+traci.vehicle.getMinGap(aID)+L_veh2+Sd1;
                        Af2=traci.vehicle.getAcceleration(aID2);
                    else
                        L_veh2=5;
                        Vf2=20;
                        Sd2=200;
                        Af2=0;
                    end
                else
                    L_veh1=5;
                    Vf1=20;
                    Sd1=200;
                    Af1=0;
                    L_veh2=5;
                    Vf2=20;
                    Sd2=200;
                    Af2=0;
                end
                [tv1]=Frontcollision(Af1,Sd1,Vf1,L_veh1,speed);
                [tv2]=Frontcollision(Af2,Sd2,Vf2,L_veh2,speed);
                tv=min(tv1,tv2);
                if tv<=3&&tv1<=tv2
                    FailLaneindex(1)=2;
                    FailLaneFrontDis(1)=Sd1;
                    FailLaneFrontVel(1)=Vf1;
                    FailLaneFrontLen(1)= L_veh1;
                elseif tv<=3
                    FailLaneindex(1)=2;
                    FailLaneFrontDis(1)=Sd2;
                    FailLaneFrontVel(1)=Vf2;
                    FailLaneFrontLen(1)= L_veh2;
                end
            end
            %紧急制动
            if usecase==178
                [aID, adist]=traci.vehicle.getLeader('S0',200);
                if isempty(aID)==0
                    [aID2, adist2]=traci.vehicle.getLeader(aID,200);
                    L_veh1=traci.vehicle.getLength(aID);
                    Vf1=traci.vehicle.getSpeed(aID);
                    Sd1=adist+traci.vehicle.getMinGap('S0')+L_veh1;
                    Af1=traci.vehicle.getAcceleration(aID);
                    if isempty(aID2)==0
                        L_veh2=traci.vehicle.getLength(aID2);
                        Vf2=traci.vehicle.getSpeed(aID2);
                        Sd2=adist2+traci.vehicle.getMinGap(aID)+L_veh2+Sd1;
                        Af2=traci.vehicle.getAcceleration(aID2);
                    else
                        L_veh2=5;
                        Vf2=20;
                        Sd2=200;
                        Af2=0;
                    end
                else
                    L_veh1=5;
                    Vf1=20;
                    Sd1=200;
                    Af1=0;
                    L_veh2=5;
                    Vf2=20;
                    Sd2=200;
                    Af2=0;
                end
                if Af1<=-3&&Af2<=-3
                    FailLaneindex(1)=2;
                    FailLaneindex(2)=2;
                    FailLaneFrontDis(1)=Sd1;
                    FailLaneFrontVel(1)=Vf1;
                    FailLaneFrontLen(1)= L_veh1;
                    FailLaneFrontDis(2)=Sd2;
                    FailLaneFrontVel(2)=Vf2;
                    FailLaneFrontLen(2)= L_veh2;
                elseif Af1<=-3
                    FailLaneindex(1)=2;
                    FailLaneFrontDis(1)=Sd1;
                    FailLaneFrontVel(1)=Vf1;
                    FailLaneFrontLen(1)= L_veh1;
                elseif Af2<=-3
                    FailLaneindex(1)=2;
                    FailLaneFrontDis(1)=Sd2;
                    FailLaneFrontVel(1)=Vf2;
                    FailLaneFrontLen(1)= L_veh2;
                end
            end
        end
              %% 场景激活
        if strcmp(current_road_ID,'7')||strcmp(current_road_ID,':8_9')||strcmp(current_road_ID,':8_11')||strcmp(current_road_ID,':8_19')% && strcmp(route{length(route)-1},'7')
            % 掉头前直行
            % 需要给出：
            if strcmp(route(find(strcmp(route,'7'))+1),'5')%第一段掉头
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
            % 需要给出：
            if strcmp(route(find(strcmp(route,'10'))+1),'9')%第二段掉头
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
            % 需要给出：
            if strcmp(route(find(strcmp(route,'gneE0'))+1),'-gneE0')%第三段掉头
                TrafficLightActive=int16(1);
                TurnAroundActive=int16(1);
            else
                TrafficLightActive=int16(1);
            end
        elseif strcmp(current_road_ID,'9')||strcmp(current_road_ID,':8_0')%十字路口右转，由上到下
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'8')%丁字路口右转
            if GlobVars.TrajPlanLaneChange.durationLaneChange==0&&GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0
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
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,':9_2')%丁字路口右转
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E13')%Y字路口左转
            TrafficLightActive=int16(1);
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,':J12_4')||strcmp(current_road_ID,':J12_7')%Y字路口左转，路口内
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,'E1')||strcmp(current_road_ID,':J2_2')%穿行人行道
            PedestrianActive=int16(1);
        elseif strcmp(current_road_ID,'E2') || strcmp(current_road_ID,':J3_0')%匝道汇入
            PedestrianActive=int16(1);
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E3') || strcmp(current_road_ID,':J4_1')%车道汇聚
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>60%6车道直行路，换道
            if GlobVars.TrajPlanLaneChange.durationLaneChange==0&&GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0
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
        elseif strcmp(current_road_ID,'E25')&&currentLanePosition>0&&(usecase==95||usecase==96||usecase==97||usecase==143)%6车道直行路，换道
            LaneChangeActive=int16(1);
%             VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E5')%J6路口前左转或右转
            if strcmp(route(find(strcmp(route,'E5'))+1),'E8')%左转
                VehicleOncomingActive=int16(1);
                TrafficLightActive=int16(1);
                PedestrianActive=int16(1);
            elseif strcmp(route(find(strcmp(route,'E5'))+1),'E7')%右转
                VehicleCrossingActive=int16(1); 
                PedestrianActive=int16(1);
            elseif strcmp(route(find(strcmp(route,'E5'))+1),'E6')%直行
                TrafficLightActive=int16(1);
            end
        elseif strcmp(current_road_ID,':J6_9')||strcmp(current_road_ID,':J6_18')%J6路口中左转
            PedestrianActive=int16(1);
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,':J6_7')
            PedestrianActive=int16(1);
            VehicleCrossingActive=int16(1);
        elseif (usecase==101||usecase==102||usecase==103)&&strcmp(current_road_ID,'2')||strcmp(current_road_ID,':gneJ11_0')||strcmp(current_road_ID,':gneJ11_1')...
                ||strcmp(current_road_ID,'a8')||strcmp(current_road_ID,':gneJ13_2')||strcmp(current_road_ID,':gneJ13_3')||strcmp(current_road_ID,'12')||strcmp(current_road_ID,':gneJ14_2')
            VehicleCrossingActive=int16(1);
        elseif strcmp(current_road_ID,'E15')||strcmp(current_road_ID,'E12')
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'E10')&&currentLanePosition>10
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'-E34')||strcmp(current_road_ID,':J30_7')||strcmp(current_road_ID,':J30_8')||strcmp(current_road_ID,':J30_6')%无红绿灯路口直行或左转或右转
            if usecase<207
                if usecase>=150&&usecase<=151
                    VehicleCrossingActive=int16(1);
                else
                    VehicleOncomingActive=int16(1);
                end
                PedestrianActive=int16(1);
            end
            if usecase==237
                 VehicleCrossingActive=int16(1);
            end
        elseif strcmp(current_road_ID,'E36')||strcmp(current_road_ID,':J34_3')||strcmp(current_road_ID,':J34_6')||strcmp(current_road_ID,':J34_2')%无红绿灯Y字路口左转或右转
            VehicleOncomingActive=int16(1);
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
            LaneChangeActive=int16(1);
        elseif strcmp(current_road_ID,'E40') || strcmp(current_road_ID,':J37_0')%三车道汇聚
            VehicleCrossingActive=int16(1); 
        elseif strcmp(current_road_ID,'E45') || strcmp(current_road_ID,':J43_0')%四车道汇聚
            VehicleCrossingActive=int16(1); 
        end    
        %% 确定掉头对向道路信息  
        if TurnAroundActive==1||GlobVars.TrajPlanTurnAround.turnAroundActive==1
            if strcmp(current_road_ID,'7')
                LaneNumber=traci.edge.getLaneNumber('5');
                oppositeLane1Width=traci.lane.getWidth('5_2');
                oppositeLane2Width=traci.lane.getWidth('5_1');
                oppositeLane3Width=traci.lane.getWidth('5_0');
                CurrentLaneWidth=traci.lane.getWidth('7_1');
                NumOfLanesOpposite=LaneNumber;
                WidthOfLanesOpposite=[oppositeLane1Width,oppositeLane2Width,oppositeLane3Width,0,0,0];
                WidthOfGap=0;
                s_turnaround_border=-6.4;
            elseif strcmp(current_road_ID,'10')
                if usecase==15||usecase==16
                    NumOfLanesOpposite=1;
                else
                    NumOfLanesOpposite=traci.edge.getLaneNumber('9');
                end
                WidthOfLanesOpposite=[traci.lane.getWidth('9_1'),traci.lane.getWidth('9_0'),0,0,0,0];
                WidthOfGap=0;
                s_turnaround_border=146.8;
            elseif strcmp(current_road_ID,'gneE0')
                LaneNumber=traci.edge.getLaneNumber('-gneE0');
                if LaneNumber==2
                    NumOfLanesOpposite=1;
                    WidthOfLanesOpposite=[3.2,0,0,0,0,0];
                    WidthOfGap=3.2;
                    s_turnaround_border=246.8;
                else
                    NumOfLanesOpposite=1;
                    WidthOfLanesOpposite=[3.2,0,0,0,0,0];
                    WidthOfGap=0;
                    s_turnaround_border=246.8;
                end
            end
        end
        if frenetflag==1
            s_turnaround_border=traci.lane.getLength([current_road_ID '_0'])+4;
        end
        %% 确定与路口的距离和停止线距离
       
        if strcmp(current_road_ID,'7')%第一个路口
            d_veh2int=-10.4-pos_s;
            if frenetflag==1
               d_veh2int= laneshape{1,2}(1,2)-laneshape{1,1}(1,2)-pos_s;
            end
        elseif usecase>31&&(strcmp(current_road_ID,':8_9')||strcmp(current_road_ID,':8_11')||strcmp(current_road_ID,':8_19'))%第一个路口
            d_veh2int=-10.4-pos_s;
            if frenetflag==1
                d_veh2int=-pos_s;
                if strcmp(current_road_ID,':8_19')
                    d_veh2int=-(traci.lane.getLength([current_road_ID '_0'])+pos_s);
                end
            end
        elseif strcmp(current_road_ID,'10')
            d_veh2int=142.8-pos_s;
            if frenetflag==1
               d_veh2int= laneshape{1,2}(1,2)-laneshape{1,1}(1,2)-pos_s;
            end
        elseif strcmp(current_road_ID,'gneE0')
            d_veh2int=242.8-pos_s;
            if frenetflag==1
               d_veh2int= laneshape{1,2}(1,2)-laneshape{1,1}(1,2)-pos_s;
            end
        elseif strcmp(current_road_ID,'9')
            d_veh2int=postion_veh(2)-10.4;
            if frenetflag==1
               d_veh2int= -(laneshape{1,2}(1,2)-laneshape{1,1}(1,2))-pos_s;
            end
        elseif strcmp(current_road_ID,'8')
            d_veh2int=postion_veh(1)-(-92.80);
            if frenetflag==1
               d_veh2int= abs(laneshape{1,1}(1,1)-laneshape{1,2}(1,1))-pos_s;
            end
        elseif strcmp(current_road_ID,'E13')
            d_veh2int=77.24-currentLanePosition-1;
        elseif strcmp(current_road_ID,'E5')%十字路口左转
            d_veh2int=traci.lane.getLength('E5_0')-currentLanePosition;
        elseif strcmp(current_road_ID,'E15')
            d_veh2int=97.54-currentLanePosition;
        elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
            d_veh2int=1000-pos_s;
            if frenetflag==1
                if strcmp(current_road_ID,'A4')
                    d_veh2int= abs(laneshape{1,1}(1,1)-laneshape{1,2}(1,1))-pos_s;
                else
                    d_veh2int=1000;
                end
            end
        end
        if d_veh2int<0&&~strcmp(current_road_ID,':8_9')%第一个路口
            d_veh2int=200;
        end
        if strcmp(current_road_ID,'E12')
            d_veh2int=traci.lane.getLength('E12_0')-currentLanePosition;
        end
        d_veh2trafficStopline=d_veh2int;
        %% 确定与汇入点距离
        if VehicleCrossingActive==1
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
            elseif strcmp(current_road_ID,'-E34')&&(usecase>=150&&usecase<=151||usecase==237)%无信灯路口右转有停车让行标志
                d_veh2converge=max([traci.lane.getLength('-E34_0')+traci.lane.getLength(':J30_6_0')-currentLanePosition 0]);
                d_veh2stopline=d_veh2converge-traci.lane.getLength(':J30_6_0');
            end
        end
        %% 确定与待转区距离
        if VehicleOncomingActive==1
            if strcmp(current_road_ID,'E13')%Y字路口左转
                d_veh2cross1(1)=max([3.2+4.18+77.24-currentLanePosition 0]);
                d_veh2waitingArea=max([d_veh2cross1(1)-3.5 0]);
            elseif strcmp(current_road_ID,':J12_4')%Y字路口中左转
                d_veh2cross1(1)=max([3.2+4.18-currentLanePosition 0]);
                d_veh2waitingArea=max([d_veh2cross1(1)-3.5 0]);
            elseif strcmp(current_road_ID,':J12_7')%Y字路口中左转
                d_veh2cross1(1)=max([3.2-currentLanePosition 0]);
                d_veh2waitingArea=max([d_veh2cross1(1)-3.5 0]);
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
        end
        %% 停车让行停止线
        if usecase>=145&&usecase<=151||(usecase>=236&&usecase<=237)
            d_veh2Signstopline=min(d_veh2stopline,d_veh2waitingArea-6);
            d_veh2waitingArea=d_veh2waitingArea-6;
        elseif usecase==152
            d_veh2waitingArea=d_veh2waitingArea-0.5;
            d_veh2Signstopline=d_veh2waitingArea;
        elseif usecase==100||usecase==206
            if strcmp(current_road_ID,'E2')||strcmp(current_road_ID,':J3_0')
                d_veh2Signstopline=d_veh2stopline;
            else
                d_veh2Signstopline=200;
            end
        else
            d_veh2Signstopline=200;
        end
       %% 确定与人行横道距离
       if PedestrianActive==1
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
       end
    %% 场景激活处理
    traci.trafficlights.getNextSwitch('9')
%         PedestrianActive
        if d_veh2trafficStopline>60
            TrafficLightActive=int16(0);
        end
        if d_veh2stopline>60
            VehicleCrossingActive=int16(0);
        end
        if d_veh2waitingArea>60
            VehicleOncomingActive=int16(0);
        end
        if s_turnaround_border-pos_s-2*TurningRadius>60
            TurnAroundActive=int16(0);
        end   
        if d_veh2Signstopline<60 && d_veh2Signstopline>=1
            VehicleCrossingActive=int16(0);
        end
        if usecase==247%Glosa场景150m激活
            if d_veh2trafficStopline<=150
                TrafficLightActive=int16(1);
            end
        end
        %% 确定信号灯相位
        if TrafficLightActivePre==0&&TrafficLightActive==1%信号灯场景激活第一帧获取路口ID缓存
            NextTLS=traci.vehicle.getNextTLS('S0') ;
            if ~isempty(NextTLS)
                tlsID=NextTLS{1,1}(1,1);
            end
        end
        TrafficLightActivePre=TrafficLightActive;%更新上一帧信号灯激活状态
        if TrafficLightActive==1 
            if strcmp(tlsID,'J6')
                %红 黄 绿 黄
                total_t=TL6phase0+TL6phase1+TL6phase2+TL6phase3;%总时间
                green_t=TL6phase2;%绿灯时间
                relativeTime=mod(i/10-TrafficLightOffset6,total_t);%红绿灯周期内的相对时间
%                 if relativeTime<TL6phase0+TL6phase1%相对时间在前两个相位
%                     trafficLightPhase=[relativeTime-(TL6phase0+TL6phase1),green_t,-(total_t-green_t),0,0,0,0,0,0,0];
%                 elseif relativeTime>=TL6phase0+TL6phase1&&relativeTime<=TL6phase0+TL6phase1+TL6phase2%相对时间在第三个相位
%                     trafficLightPhase=[TL6phase0+TL6phase1+TL6phase2-relativeTime,-(total_t-green_t),green_t,0,0,0,0,0,0,0];
%                 elseif relativeTime>TL6phase0+TL6phase1+TL6phase2%相对时间在第四个相位
%                     trafficLightPhase=[relativeTime-(total_t+TL6phase0+TL6phase1),green_t,-(total_t-green_t),0,0,0,0,0,0,0];
%                 end
                if relativeTime<TL6phase0%相对时间在第一个相位
                    trafficLightPhase=[relativeTime-TL6phase0,-TL6phase1,TL6phase2,-TL6phase3,0,0,0,0,0,0];
                     greenLight=0;
                elseif relativeTime<TL6phase0+TL6phase1%相对时间在第二相位
                    trafficLightPhase=[relativeTime-(TL6phase0+TL6phase1),TL6phase2,-TL6phase3,-TL6phase0,0,0,0,0,0];
                     greenLight=2;
                elseif relativeTime<=TL6phase0+TL6phase1+TL6phase2%相对时间在第三个相位
                    trafficLightPhase=[TL6phase0+TL6phase1+TL6phase2-relativeTime,-TL6phase3,-TL6phase0,-TL6phase1,0,0,0,0,0,0];
                     greenLight=1;
                elseif relativeTime>TL6phase0+TL6phase1+TL6phase2%相对时间在第四个相位
                    trafficLightPhase=[relativeTime-(total_t),-TL6phase0,-TL6phase1,TL6phase2,0,0,0,0,0,0];
                     greenLight=2;
                end
                %无需算黄灯
%                 if trafficLightPhase(1)>0
%                     greenLight=1;
%                     time2nextSwitch=trafficLightPhase(1);
%                 else
%                     greenLight=0;
%                     time2nextSwitch=0;
%                 end
            else
                %绿 黄 红 黄 42 3 42 3
                if strcmp(tlsID,'0')==1
                    tlsID={'J12'};  
                end
                tlsOffset=str2double(tlLogic(strcmp(tlLogic(:,1),tlsID),2));
                relativeTime=mod(i/10-tlsOffset,90);
                if relativeTime<42
                    trafficLightPhase=[42-relativeTime,-3,-42,-3,0,0,0,0,0,0];
                    greenLight=1;
                elseif relativeTime<42+3
                    trafficLightPhase=[-(42+3-relativeTime),-42,-3,42,0,0,0,0,0,0];
                    greenLight=2;
                elseif relativeTime>=42+3+42
                    trafficLightPhase=[-(42+3+42-relativeTime),-3,42,-3,0,0,0,0,0,0];
                    greenLight=0;
                else
                    trafficLightPhase=[-(42+3+42+3-relativeTime),42,-3,-42,0,0,0,0,0,0];
                    greenLight=2;
                end
                %需考虑黄灯，（信号灯通行AEB判断需）
%                 if traci.trafficlights.getPhase(tlsID{1,1})==0
%                     greenLight=1;
%                     time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
%                 elseif traci.trafficlights.getPhase(tlsID{1,1})==1
%                     greenLight=2;
%                     time2nextSwitch=(420-mod(i-TrafficLightOffset*10,900))/10;
%                 else
%                     greenLight=0;
%                     time2nextSwitch=0;
%                 end
            end    
        end
        %红灯认为控制
        if usecase==60
            if i>=790+1
                greenLight=0;
            end
        elseif usecase==235
            if i>=791+2
                greenLight=0;
            end
        end
       %% 确定行人位置、速度
       if PedestrianActive==1
           if strcmp(current_road_ID,'E1')||strcmp(current_road_ID,':J2_2')||strcmp(current_road_ID,'E2')
               if (usecase>=126&&usecase<=130)||(usecase>=218&&usecase<=219)||usecase==232
                   [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('E1_0',':J2_w0','E10','E12',pos_s);
               else
                   [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('E1_0',':J2_w0',[],[],pos_s);
               end
               if frenetflag==1
                   s_ped=s_ped-laneshape{1,1}(1,2);
               end
           elseif (strcmp(current_road_ID,'E5')||strcmp(current_road_ID,':J6_9')||strcmp(current_road_ID,':J6_18'))&&strcmp(route(find(strcmp(route,'E5'))+1),'E8')%J6路口左转
               [pos_scompare,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('E5_0',':J6_9_0',':J6_18_0',':J6_c1',pos_s,current_lane_ID,'turnright');%左转
               if frenetflag==1
                   if strcmp(current_road_ID,':J6_9')
                       s_ped=s_ped-traci.lane.getLength('E5_0')+100;
                   elseif strcmp(current_road_ID,':J6_18')
                       s_ped=s_ped-traci.lane.getLength('E5_0')-traci.lane.getLength(':J6_9_0')+100;
                   end
               else
                  pos_s = pos_scompare;
               end
           elseif (strcmp(current_road_ID,'E5')||strcmp(current_road_ID,':J6_7'))&&strcmp(route(find(strcmp(route,'E5'))+1),'E7')%J6路口右转
               [pos_scompare,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('E5_0',':J6_7_0',':J6_17_0',':J6_c0',pos_s,current_lane_ID,'turnright');%右转
               if frenetflag==1
                   if strcmp(current_road_ID,'E5')
                       s_ped=s_ped+100;
                   elseif strcmp(current_road_ID,':J6_7')
                       s_ped=s_ped-traci.lane.getLength('E5_0')+100;
                   end
               else
                  pos_s = pos_scompare;
               end
           elseif (strcmp(current_road_ID,'-E34')||(strcmp(current_road_ID,':J30_7')&&currentLanePosition<w_cross)||(strcmp(current_road_ID,':J30_8')...
                   &&currentLanePosition<w_cross)||(strcmp(current_road_ID,':J30_6')&&currentLanePosition<w_cross)&&usecase~=131)&&usecase>=112&&usecase<145%无红绿灯路口直行或左转或右转
               [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform('-E34_0',':J30_c2',[],[],pos_s);
               if frenetflag==1
                   s_ped=s_ped-laneshape{1,1}(1,2);
               end
           elseif strcmp(current_road_ID,':J30_7')&&usecase>=112&&usecase<145%无红绿灯路口直行
               [pos_s,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform(':J30_7_0',':J30_c0',[],[],pos_s);
               if frenetflag==1
                   s_ped=s_ped-laneshape{1,1}(1,2);
               end
           elseif strcmp(current_road_ID,':J30_8')&&usecase>=112&&usecase<145%无红绿灯路口左转
               [pos_scompare,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('-E34_0',':J30_8_0',[],':J30_c3',pos_s,current_lane_ID,'turnleft');
           elseif strcmp(current_road_ID,':J30_6')&&usecase>=112&&usecase<145%无红绿灯路口右转
               [pos_scompare,s_ped,l_ped,v_ped,psi_ped]=AvoPedInform3('-E34_0',':J30_6_0',[],':J30_c1',pos_s,current_lane_ID,'turnright');%右转
               if frenetflag==1
                   s_ped=s_ped-traci.lane.getLength('-E34_0')+350;
               else
                   pos_s = pos_scompare;
               end
           end
       end
        %% 搜寻匝道汇入时主路上车辆
        if VehicleCrossingActive==1
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
                %             CurrentLaneFrontVel=v_a;
                %             CurrentLaneFrontDis=s_a;
                %             CurrentLaneFrontLen=l_a;
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
                %             CurrentLaneFrontVel=v_a;
                %             CurrentLaneFrontDis=s_a;
                %             CurrentLaneFrontLen=l_a;
                [s_rampf,v_rampf,l_rampf]=RampVehInform3('E3_0',':J4_1_0',d_veh2converge);
                CurrentLaneFrontDisAvoidVehicle=s_rampf;
                CurrentLaneFrontVelAvoidVehicle=v_rampf;
                CurrentLaneFrontLenAvoidVehicle=l_rampf;
                if s_rampf<=s_a
                    CurrentLaneFrontVel=v_rampf;
                    CurrentLaneFrontDis=s_rampf;
                    CurrentLaneFrontLen=l_rampf;
                else
                    CurrentLaneFrontVel=v_a;
                    CurrentLaneFrontDis=s_a;
                    CurrentLaneFrontLen=l_a;
                end
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
                if s_rampf<=s_a
                    CurrentLaneFrontDisAvoidVehicle=s_rampf;
                    CurrentLaneFrontVelAvoidVehicle=v_rampf;
                    CurrentLaneFrontLenAvoidVehicle=l_rampf;
                else
                    CurrentLaneFrontDisAvoidVehicle=s_a;
                    CurrentLaneFrontVelAvoidVehicle=v_a;
                    CurrentLaneFrontLenAvoidVehicle=l_a;
                end
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
        end
        %% 搜寻对向车
        if VehicleOncomingActive==1
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
        end
        %% 搜寻掉头对向车
        if TurnAroundActive==1||GlobVars.TrajPlanTurnAround.turnAroundActive==1
            if postion_veh(2)>-50&&postion_veh(2)<10&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241))%第一个路口
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
                    IndexOfLaneOppositeCar(1:length(VehiclesLane(:,1)))=VehiclesLane(:,1);
                    SpeedOppositeCar(1:length(VehiclesLane(:,3)))=VehiclesLane(:,3);
                    PosSOppositeCar(1:length(VehiclesLane(:,2)))=VehiclesLane(:,2);
                    LengthOppositeCar(1:length(VehiclesLane(:,4)))=VehiclesLane(:,4);
                end
                
            elseif postion_veh(2)>100&&postion_veh(2)<160&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241)||usecase==100)%第二个路口
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
                    IndexOfLaneOppositeCar(1:length(VehiclesLane(:,1)))=VehiclesLane(:,1);
                    SpeedOppositeCar(1:length(VehiclesLane(:,3)))=VehiclesLane(:,3);
                    PosSOppositeCar(1:length(VehiclesLane(:,2)))=VehiclesLane(:,2);
                    LengthOppositeCar(1:length(VehiclesLane(:,4)))=VehiclesLane(:,4);
                end
            elseif postion_veh(2)>200&&postion_veh(2)<260&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241))%第三个路口
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
                    IndexOfLaneOppositeCar(1:length(VehiclesLane(:,1)))=VehiclesLane(:,1);
                    SpeedOppositeCar(1:length(VehiclesLane(:,3)))=VehiclesLane(:,3);
                    PosSOppositeCar(1:length(VehiclesLane(:,2)))=VehiclesLane(:,2);
                    LengthOppositeCar(1:length(VehiclesLane(:,4)))=VehiclesLane(:,4);
                end
            end
        end
        if frenetflag==1
            PosSOppositeCar=PosSOppositeCar+pos_s-postion_veh(2);
        end
        %% 搜寻掉头前车道环境车
        if TurnAroundActive==1||GlobVars.TrajPlanTurnAround.turnAroundActive==1
            if postion_veh(2)>-50&&postion_veh(2)<10&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241))%第一个路口
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
                    IndexOfLaneCodirectCar(1:length(VehicleInformLane(:,1)))=VehicleInformLane(:,1);
                    SpeedCodirectCar(1:length(VehicleInformLane(:,3)))=VehicleInformLane(:,3);
                    PosSCodirectCar(1:length(VehicleInformLane(:,2)))=VehicleInformLane(:,2);
                    LengthCodirectCar(1:length(VehicleInformLane(:,4)))=VehicleInformLane(:,4);
                end
            elseif postion_veh(2)>100&&postion_veh(2)<160&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241))%第二个路口
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
                    IndexOfLaneCodirectCar(1:length(VehicleInformLane(:,1)))=VehicleInformLane(:,1);
                    SpeedCodirectCar(1:length(VehicleInformLane(:,3)))=VehicleInformLane(:,3);
                    PosSCodirectCar(1:length(VehicleInformLane(:,2)))=VehicleInformLane(:,2);
                    LengthCodirectCar(1:length(VehicleInformLane(:,4)))=VehicleInformLane(:,4);
                end
            elseif postion_veh(2)>200&&postion_veh(2)<260&&(usecase<=31||(usecase>=168&&usecase<=173)||(usecase>=239&&usecase<=241))%第三个路口
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
                    SpeedCodirectCar(1:length(VehiclesLane1(:,3)))=VehiclesLane1(:,3);
                    PosSCodirectCar(1:length(VehiclesLane1(:,2)))=VehiclesLane1(:,2);
                    LengthCodirectCar(1:length(VehicleInformLane(:,4)))=VehicleInformLane(:,4);
                end
            end
        end
        if frenetflag==1
            PosSCodirectCar=PosSCodirectCar+pos_s-postion_veh(2);
        end
        %% 搜寻左右车
        if LaneChangeActive==1
            if strcmp(current_road_ID,'E25')
                [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
                    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumofLane,lane,current_road_ID,currentLanePosition);
            elseif strcmp(current_road_ID,'E15')
                NumofLane=2;
                % 搜寻右车
                lane=max([0 lane]);
                lane=min([NumofLane-1 lane]);
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
                LeftLaneIndex=['E15_' num2str(round(min([NumofLane-1 lane+1])))];
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
                    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumofLane,lane,current_road_ID,currentLanePosition);
                if usecase==161
                    LeftLaneFrontLen=5;
                end
            elseif strcmp(current_road_ID,'E10')
                [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis...
                    ,RightLaneFrontLen,RightLaneBehindLen,LeftLaneFrontLen,LeftLaneBehindLen]=LaneChangeCars(NumofLane,lane,current_road_ID,currentLanePosition);
            elseif strcmp(current_road_ID,'A0')||strcmp(current_road_ID,'A1')||strcmp(current_road_ID,'A2')||strcmp(current_road_ID,'A3')||strcmp(current_road_ID,'A4')
                %             [RightLaneFrontVel,RightLaneFrontDis,RightLaneBehindVel,RightLaneBehindDis,LeftLaneFrontVel,LeftLaneFrontDis,LeftLaneBehindVel,LeftLaneBehindDis]=...
                %     LaneChangeCars(NumOfLanes,lane,current_road_ID,currentLanePosition);
                %            NumOfLanes=6;
                % 搜寻右车
                lane=max([0 lane]);
                lane=min([NumofLane-1 lane]);
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
                LeftLaneIndex1=['A0_' num2str(round(min([NumofLane-1 lane+1])))];
                LeftLaneIndex2=['A1_' num2str(round(min([NumofLane-1 lane+1])))];
                LeftLaneIndex3=['A2_' num2str(round(min([NumofLane-1 lane+1])))];
                LeftLaneIndex4=['A3_' num2str(round(min([NumofLane-1 lane+1])))];
                LeftLaneIndex5=['A4_' num2str(round(min([NumofLane-1 lane+1])))];
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
        end
        %% 档位变换
        CurrentGear=Decision.TargetGear;%掉头时换挡
        %% 入参
        BasicsInfo.currentLaneFrontDis=CurrentLaneFrontDis;
        BasicsInfo.currentLaneFrontVel=CurrentLaneFrontVel;
        BasicsInfo.currentLaneFrontLen=CurrentLaneFrontLen;
        BasicsInfo.pos_s=pos_s;
        BasicsInfo.pos_l=pos_l;
        BasicsInfo.pos_psi=pos_psi;
        BasicsInfo.pos_l_CurrentLane=pos_l_CurrentLane;
        BasicsInfo.currentLaneIndex=int16(CurrentLaneIndex);
        BasicsInfo.widthOfLanes=WidthOfLanes;
        BasicsInfo.targetLaneIndex=int16(TargetLaneIndex);
        BasicsInfo.v_max=v_max;
        BasicsInfo.goalLaneIndex=GoalLaneIndex;
        BasicsInfo.d_veh2goal=d_veh2goal;
        BasicsInfo.sampleTime=SampleTime;
        ChassisInfo.speed=double(speed);
        ChassisInfo.currentGear=int16(CurrentGear);
        AvoFailVehInfo.lanesWithFail=LanesWithFail;
        AvoFailVehInfo.failLaneindex=FailLaneindex;
        AvoFailVehInfo.failLaneFrontDis=FailLaneFrontDis;
        AvoFailVehInfo.failLaneFrontVel=FailLaneFrontVel;
        AvoFailVehInfo.failLaneFrontLen=FailLaneFrontLen;
        LaneChangeInfo.d_veh2int = d_veh2int;
        LaneChangeInfo.leftLaneBehindDis=LeftLaneBehindDis;
        LaneChangeInfo.leftLaneBehindVel=LeftLaneBehindVel;     
        LaneChangeInfo.leftLaneFrontDis=LeftLaneFrontDis;
        LaneChangeInfo.leftLaneFrontVel=LeftLaneFrontVel; 
        LaneChangeInfo.rightLaneBehindDis=RightLaneBehindDis;
        LaneChangeInfo.rightLaneBehindVel=RightLaneBehindVel;
        LaneChangeInfo.rightLaneFrontDis=RightLaneFrontDis;
        LaneChangeInfo.rightLaneFrontVel=RightLaneFrontVel;
        LaneChangeInfo.leftLaneBehindLen=LeftLaneBehindLen;
        LaneChangeInfo.leftLaneFrontLen=LeftLaneFrontLen;
        LaneChangeInfo.rightLaneBehindLen=RightLaneBehindLen;
        LaneChangeInfo.rightLaneFrontLen=RightLaneFrontLen;
        AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(1)=TargetLaneBehindDisAvoidVehicle;
        AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(1)=TargetLaneBehindVelAvoidVehicle;
        AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(1)=TargetLaneBehindLenAvoidVehicle;
        AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(1)=TargetLaneFrontDisAvoidVehicle;
        AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(1)=TargetLaneFrontVelAvoidVehicle;
        AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(1)=TargetLaneFrontLenAvoidVehicle;
        AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(2)=TargetLaneBehindDisAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(2)=TargetLaneBehindVelAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(2)=TargetLaneBehindLenAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(2)=TargetLaneFrontDisAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(2)=TargetLaneFrontVelAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(2)=TargetLaneFrontLenAvoidVehicle1;
        AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(3)=TargetLaneBehindDisAvoidVehicle2;
        AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle(3)=TargetLaneBehindVelAvoidVehicle2;
        AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle(3)=TargetLaneBehindLenAvoidVehicle2;
        AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(3)=TargetLaneFrontDisAvoidVehicle2;
        AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(3)=TargetLaneFrontVelAvoidVehicle2;
        AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle(3)=TargetLaneFrontLenAvoidVehicle2;
        AvoMainRoVehInfo.d_veh2converge=d_veh2converge;
        AvoMainRoVehInfo.d_veh2stopline=d_veh2stopline;
        StopSignInfo.d_veh2stopline=d_veh2Signstopline;
        AvoOncomingVehInfo.d_veh2waitingArea=d_veh2waitingArea;
        AvoOncomingVehInfo.s_veh=s_veh1;
        AvoOncomingVehInfo.v_veh=v_veh1;
        AvoOncomingVehInfo.l_veh=s_veh_length;
        AvoOncomingVehInfo.d_veh2conflict=d_veh2cross1;
        AvoOncomingVehInfo.s_vehapostrophe=s_veh1apostrophe1;
        AvoOncomingVehInfo.l_vehapostrophe=s_vehapostrophe_length;
        AvoPedInfo.d_veh2cross=d_veh2cross;
        AvoPedInfo.w_cross=w_cross;
        AvoPedInfo.s_ped=s_ped;
        AvoPedInfo.v_ped=v_ped;
        AvoPedInfo.l_ped=l_ped;
        AvoPedInfo.psi_ped=psi_ped;
        TrafficLightInfo.greenLight=greenLight;
%         TrafficLightInfo.time2nextSwitch=time2nextSwitch;
        TrafficLightInfo.d_veh2stopline=d_veh2trafficStopline;
        TrafficLightInfo.phase=trafficLightPhase;%zeros(1,10);
        TurnAroundInfo.numOfLanesOpposite=int16(NumOfLanesOpposite);
        TurnAroundInfo.widthOfLanesOpposite=WidthOfLanesOpposite;
        TurnAroundInfo.widthOfGap=double(WidthOfGap);
        TurnAroundInfo.s_turnaround_border=double(s_turnaround_border);
        TurnAroundInfo.indexOfLaneOppositeCar=int16(IndexOfLaneOppositeCar);
        TurnAroundInfo.speedOppositeCar=SpeedOppositeCar;
        TurnAroundInfo.posSOppositeCar=PosSOppositeCar;
        TurnAroundInfo.indexOfLaneCodirectCar=int16(IndexOfLaneCodirectCar);
        TurnAroundInfo.speedCodirectCar=SpeedCodirectCar;
        TurnAroundInfo.posSCodirectCar=PosSCodirectCar;
        TurnAroundInfo.lengthOppositeCar=LengthOppositeCar;
        TurnAroundInfo.lengthCodirectCar=LengthCodirectCar;
        %% UrbanPlanner
        [Trajectory,Decision,Refline,GlobVars]=UrbanPlanner(BasicsInfo,ChassisInfo,LaneChangeInfo,AvoMainRoVehInfo,AvoPedInfo,TrafficLightInfo,AvoOncomingVehInfo,...,
            AvoFailVehInfo,TurnAroundInfo,StopSignInfo,LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive,GlosaActive,PlannerLevel,GlobVars,CalibrationVars,Parameters);
%           pause(0.05)
%         if frenetflag==1&&PlannerLevel==1%画轨迹点
%             traj_x_array=zeros([1 80]);
%             traj_y_array=zeros([1 80]);
%             for traj_index=1:80
%                 [traj_x,traj_y,~]=frenet2XY(Trajectory.traj_s(traj_index),Trajectory.traj_l(traj_index),Trajectory.traj_psi(traj_index),nodelist_s,laneshape);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x traj_y+0.1]});
%                 traj_x_array(traj_index)=traj_x;
%                 traj_y_array(traj_index)=traj_y;
% %                 plot(traj_x_array,traj_y_array,'r*')
%             end
%         end
        % Decision
        if PlannerLevel==3
            TargetVelocity=Decision.TargetSpeed*3.6;
        end
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
                traci.vehicle.changeLane('S0',min(lane+1,NumofLane-1),2);
                set(gcf,'CurrentCharacter','r')
            elseif strcmpi(get(gcf,'CurrentCharacter'),'f')
                traci.vehicle.changeLane('S0',max(lane-1,0),2);
                set(gcf,'CurrentCharacter','r')
            else
                targetspeed=speed;
            end
            traci.vehicle.setSpeed('S0',targetspeed);
        elseif PlannerLevel==3
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
                        if speed>TargetVelocity/3.6
                            traci.vehicle.setSpeed('S0',speed-1.5*1);
                        else
                            traci.vehicle.setSpeed('S0',TargetVelocity/3.6);
                        end
                    else
                        traci.vehicle.setSpeed('S0',TargetVelocity/3.6);
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
                if TimeResponse2>=5&&usecase>=208&&usecase<=211%&&manual~=3
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
        elseif PlannerLevel==2
%             traci.vehicle.setSpeedMode('S0', 31);
            %横向指令
            if Decision.LaneChange==1
                traci.vehicle.changeLane('S0',lane+1,2);
            elseif Decision.LaneChange==2
                traci.vehicle.changeLane('S0',lane-1,2);
            else
                traci.vehicle.changeLane('S0',lane,2);
            end
            %纵向指令
            if Refline.NumRefLaneTurnAround~=0%掉头激活画一帧参考线
                if frenetflag==1
                     for traj_index=1:Refline.NumRefLaneTurnAround-1
                        [traj_x,traj_y,~]=frenet2XY(Refline.SRefLaneTurnAround(traj_index),Refline.LRefLaneTurnAround(traj_index),0,nodelist_s,laneshape);
                        [traj_xnext,traj_ynext,~]=frenet2XY(Refline.SRefLaneTurnAround(traj_index+1),Refline.LRefLaneTurnAround(traj_index+1),0,nodelist_s,laneshape);
                        traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_xnext traj_ynext]});
                    end
                else
                    for traj_index=1:Refline.NumRefLaneTurnAround-1
                        traj_x=101.6-Refline.LRefLaneTurnAround(traj_index);
                        traj_y=Refline.SRefLaneTurnAround(traj_index);
                        traj_xnext=101.6-Refline.LRefLaneTurnAround(traj_index+1);
                        traj_ynext=Refline.SRefLaneTurnAround(traj_index+1);
                        traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_xnext traj_ynext]});
                    end
                end
            end
            if Decision.TurnAroundState==1&&pos_s>GlobVars.TrajPlanTurnAround.posCircle(1)-TurningRadius
                turnAroundMoveToflag=1;
            end
            if Refline.TurnAroundReflineState==0&&TurnAroundReflineState==1%掉头参考线被更新 画一帧掉头后参考线
                turnAroundMoveToflag=0;
                for traj_index=1:80%参考线清零
                    traj_x=101.6-Refline.LRefLaneTurnAround(traj_index);
                    traj_y=Refline.SRefLaneTurnAround(traj_index);
                    traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x traj_y]});
                end
                laneshape=traci.lane.getShape(current_lane_ID );
                traci.polygon.setShape('traj1',{[laneshape{1,1}(1,1) laneshape{1,1}(1,2)] [laneshape{1,2}(1,1) laneshape{1,2}(1,2)]});
            end
            if turnAroundMoveToflag==1%moveto-vehicle
                PosCircle1= GlobVars.TrajPlanTurnAround.posCircle;
                PosCircle2= GlobVars.TrajPlanTurnAround.posCircle2;
                %计算自车在掉头路径中已走过路径
                if pos_s-PosCircle1(1)>0&&pos_l<PosCircle1(2)
                    passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
                    passedPerimeter=(pi/2+passedAngle)*TurningRadius;
                elseif pos_l<=PosCircle2(2)&&pos_l>=PosCircle1(2)
                    passedPerimeter=pos_l-PosCircle1(2)+TurningRadius*pi/2;
                elseif pos_l>PosCircle2(2)
                    passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
                    if passedAngle<0
                        passedPerimeter=PosCircle2(2)-PosCircle1(2)+TurningRadius*pi+PosCircle2(1)-pos_s;
                    else
                        passedPerimeter=passedAngle*TurningRadius+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2;
                    end
                else
                    passedPerimeter=pos_s-PosCircle1(1);
                end
                %计算下一帧目标车速
                if Decision.AEBactive>0
                    targetSpeed=max(0,speed+(-4)*SampleTime);
                    a_soll_TrajPlanTurnAround=-4;
                elseif Decision.Wait>0
                    targetaccel= ACC(v_max,0,Decision.WaitDistance+4-0.1,speed,1,CalibrationVars);
                    targetSpeed=max(0,speed+targetaccel*SampleTime);
                    a_soll_TrajPlanTurnAround=targetaccel;
                else
                    targetSpeed=Decision.TargetSpeed;
                    a_soll_TrajPlanTurnAround=Decision.a_soll;
                end
                %计算自车至下一帧时走过的路径及圆弧角度
                if targetSpeed==0
                    adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
                else
                    adavancedPerimeter=(targetSpeed+speed)*SampleTime/2;
                end
                targetPerimeter=adavancedPerimeter+passedPerimeter;
                if targetPerimeter<pi*TurningRadius/2
                    targetAngle=targetPerimeter/TurningRadius-pi/2;
                elseif targetPerimeter<=pi*TurningRadius/2+PosCircle2(2)-PosCircle1(2)&&targetPerimeter>=pi*TurningRadius/2
                    targetAngle=0;
                else
                    targetAngle=(targetPerimeter-(PosCircle2(2)-PosCircle1(2)))/TurningRadius-pi/2;
                end
                %计算自车一下帧在掉头路径中位置
                if targetAngle<=-pi/2%第一段直行位置
                    target_s=pos_s+adavancedPerimeter;
                    target_l=pos_l_CurrentLane;
                    target_psi=90;
                elseif targetAngle<0%第一段圆弧位置
                    target_s=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                    target_l=PosCircle1(2)+sin(targetAngle)*TurningRadius;
                    target_psi=-targetAngle*180/pi;
                elseif targetAngle==0%中间直行位置
                    target_s=PosCircle1(1)+TurningRadius;
                    target_l=PosCircle1(2)+targetPerimeter-pi*TurningRadius/2;
                    target_psi=-targetAngle*180/pi;
                elseif targetAngle<=pi/2%第二段圆弧位置
                    target_s=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                    target_l=PosCircle2(2)+sin(targetAngle)*TurningRadius;
                    target_psi=-targetAngle*180/pi;
                else%过渡路径位置
                    target_s=PosCircle1(1)-(passedPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2)+adavancedPerimeter*cos(atan(0.1)));
                    target_l=PosCircle2(2)+TurningRadius-(passedPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2)+adavancedPerimeter*cos(atan(0.1)))*0.1;
                    if target_l<=GlobVars.TrajPlanTurnAround.reflineLend%车头走完过度段
                        target_l=GlobVars.TrajPlanTurnAround.reflineLend;
                        target_psi=-90;
                    else
                        target_psi=-90-atand(0.1);
                    end
                end
                %自车移动至该位置
                if frenetflag==1
                    [pos_x,pos_y,pos_yaw]=frenet2XY(target_s,target_l,target_psi,nodelist_s,laneshape);
                    traci.vehicle.moveToXY('S0',current_road_ID, 2, pos_x, pos_y,pos_yaw,2);
                else
                    traci.vehicle.moveToXY('S0','12', 2, 101.6-target_l, target_s,target_psi-90,2);
                end
                traci.vehicle.setSpeed('S0',Decision.TargetSpeed);
            else %setSpeed-vehicle
                if Decision.Wait>0
                    targetaccel= ACC(v_max,0,Decision.WaitDistance+4-0.1,speed,1,CalibrationVars);
                    traci.vehicle.setSpeed('S0',max(0,speed+targetaccel*SampleTime));
                else
                    if Decision.PedestrianState|| Decision.TrafficLightState||Decision.VehicleCrossingState||Decision.VehicleOncomingState||Decision.StopSignState||Decision.PullOverState||Decision.SlowDown==6||Decision.TurnAroundState
                        targetaccel=(Decision.TargetSpeed-speed)/SampleTime;
                    else
%                      if usecase>=208&&usecase<=211
%                         if  Decision.SlowDown==6
%                             targetaccel=(Decision.TargetSpeed-speed)/SampleTime;
%                         else
%                             targetaccel=ACC(50,20,200,speed,0,CalibrationVars);
%                         end
%                      else
                        targetaccel=ACC(Decision.TargetSpeed,CurrentLaneFrontVel,CurrentLaneFrontDis-CurrentLaneFrontLen,speed,0,CalibrationVars);
%                      end
                    end
                    traci.vehicle.setSpeed('S0',speed+targetaccel*SampleTime);
                end
            end
            %接管指令
            if Decision.AEBactive>0
                targetspeed=max(0,speed+(-4)*SampleTime);
                traci.vehicle.setSpeed('S0',targetspeed);
            end
%         elseif Decision.a_soll~=100&&Decision.LaneChange==0
        elseif GlobVars.TrajPlanLaneChange.durationLaneChange==0&&GlobVars.TrajPlanTurnAround.turnAroundActive==0&&GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan==0&&Decision.LaneChange==0
            if (Trajectory.traj_vs(2)-speed)/SampleTime<-4-0.000001||(Trajectory.traj_vs(2)-speed)/SampleTime>2.5+0.000001
                (Trajectory.traj_vs(2)-speed)/SampleTime
            end
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
        elseif (GlobVars.TrajPlanLaneChange.durationLaneChange~=0||GlobVars.TrajPlanTurnAround.turnAroundActive~=0||GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan~=0)&&frenetflag==1
              [pos_x,pos_y,pos_yaw]=frenet2XY(Trajectory.traj_s(2),Trajectory.traj_l(2),Trajectory.traj_psi(2),nodelist_s,laneshape);
            traci.vehicle.moveToXY('S0',current_road_ID, 2, pos_x, pos_y,pos_yaw,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));       
        elseif strcmp(current_road_ID,'E25')||strcmp(current_road_ID,'E12')||strcmp(current_road_ID,'E10')%避让故障车换道
            traci.vehicle.moveToXY('S0',current_road_ID, 2, laneshape{1,1}(1,1)-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));       
%             for traj_index=1:80
%                 traj_x=laneshape{1,1}(1,1)-Trajectory.traj_l(traj_index);
%                 traj_y=Trajectory.traj_s(traj_index);
%                 traci.polygon.setShape(['traj' num2str(traj_index)],{[traj_x traj_y] [traj_x+0.1 traj_y+0.1]});
%             end   
        elseif strcmp(current_road_ID,'8')||strcmp(current_road_ID,'-E14')%T字路口右转前换道
            traci.vehicle.moveToXY('S0',current_road_ID, 2, -Trajectory.traj_s(2),laneshape{1,1}(1,2)-Trajectory.traj_l(2),Trajectory.traj_psi(2)-180,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
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
        elseif  TurnAroundMoveTo==1||GlobVars.TrajPlanTurnAround.turnAroundActive%掉头
            traci.vehicle.moveToXY('S0','12', 2, 101.6-Trajectory.traj_l(2), Trajectory.traj_s(2),Trajectory.traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
        elseif strcmp(current_road_ID,'5')||strcmp(current_road_ID,'9')||strcmp(current_road_ID,'-gneE0')%掉头后路段
            traci.vehicle.moveToXY('S0',current_road_ID, 2, laneshape{1,1}(1,1)+Trajectory.traj_l(2),-Trajectory.traj_s(2),Trajectory.traj_psi(2)+90,2);
            traci.vehicle.setSpeed('S0',sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2));
        end
        TurnAroundReflineState=Refline.TurnAroundReflineState;%缓存参考线状态
        %         if Desider.wait_turnAround==1 || Desider.wait_TrafficLight==1 || Desider.wait_AvoidVehicle==1 || Desider.wait_ped==1|| Desider.wait_avoidOncomingVehicle==1
        %             sound(sin(2*pi*25*(1:4000)/100));
        %         end      
        %% 画图
        if manual==2||(usecase>=201&&usecase<=205)
%             w(1,i)=double(Decision.LaneChange);
            w(1,(i-1)/(SampleTime*10))=double(Decision.LaneChange);
            set(p(1),'XData',t,'YData',w(1,:))
            %         set(p(2),'XData',t,'YData',w(2,:))
            %         set(p(3),'XData',t,'YData',w(3,:))
            %         set(p(4),'XData',t,'YData',w(4,:))
            %         set(p(5),'XData',t,'YData',w(5,:))
            set(g,'String',['speed' 32 '=' 32 num2str(speed*3.6) 'km/h'],'Position',[x+12 2.3 0])
            set(wait_text,'String',['wait' 32 '=' 32 num2str(Decision.Wait)],'Position',[x+1 2.6 0])
            if Decision.Wait==0
                set(waitDist_text,'String',['dist' 32 32 '=' 32 'NaN'],'Position',[x+1 2.3 0])
            else
                set(waitDist_text,'String',['dist' 32 32 '=' 32 num2str(Decision.WaitDistance) 'm'],'Position',[x+1 2.3 0])
            end
            set(slowdown_text,'String',['slowd' 32 32 '=' 32 num2str(Decision.SlowDown)],'Position',[x+12 2.9 0])
            if Decision.TargetSpeed==-20
                set(targetspeed_text,'String',['targV' 32 32 '=' 32 'NaN'],'Position',[x+12 2.6 0])
            else
                set(targetspeed_text,'String',['targV' 32 32 '=' 32 num2str(round(Decision.TargetSpeed*3.6*100)/100) 'km/h'],'Position',[x+12 2.6 0])
            end
            set(start_text,'String',['start' 32 '=' 32 num2str(Decision.Start)],'Position',[x+1 2.9 0])
            set(AEB_text,'String',['AEB' 32 '=' 32 num2str(Decision.AEBactive)],'Position',[x+30 2.3 0])
            set(Title,'String',['TargetLane=' num2str(TargetLaneIndex) 32 32 32 32 'CurrentLane=' num2str(CurrentLaneIndex)]);
            x=x+ SampleTime;
            axis([x x+50 -0.5 3]);
            drawnow
%---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        elseif manual==4
            w(1,(i-1)/(SampleTime*10))=double(Decision.LaneChange)/2;
            set(p(1),'XData',t,'YData',w(1,:))
             set(g,'String',['实际速度' 32 '=' 32 num2str(round(speed*3.6*10)/10) 'km/h'],'Position',[x+25 1.7 0])
            set(wait_text,'String','停车距离','Position',[x+12 2.0 0],'FontSize',12)
            if Decision.Wait==0
                set(waitDist_text,'String',[32 'NaN'],'Position',[x+12 1.7 0])
            else
                set(waitDist_text,'String',[32 num2str(Decision.WaitDistance) 'm'],'Position',[x+12 1.7 0])
            end
            if Decision.SlowDown>0
                set(slowdown_text,'String','减速','Position',[x+1 2.3 0])
            elseif Decision.Wait>0
                set(slowdown_text,'String','停车','Position',[x+1 2.3 0])
            elseif Decision.Start>0
                set(slowdown_text,'String','起步','Position',[x+1 2.3 0])
            else
                set(slowdown_text,'String','NaN','Position',[x+1 2.3 0])
            end
            
            if Decision.Wait>0
                set(targetspeed_text,'String',[32 'NaN'],'Position',[x+12 2.3 0])
            else
%                 if PedestrianActive==1||TrafficLightActive==1||VehicleCrossingActive==1||VehicleOncomingActive==1
%                     targV=round((sqrt((Trajectory.traj_vs(2)).^2+(Trajectory.traj_vl(2)).^2)*3.6)*10)/10;
%                 else
%                     targV=50;
%                 end
                set(targetspeed_text,'String',[32 num2str(round(Decision.TargetSpeed*3.6*100)/100) 'km/h'],'Position',[x+12 2.3 0])
            end
%             set(start_text,'String',['start' 32 '=' 32 num2str(Decision.Start)],'Position',[x+1 1.7 0])
            set(AEB_text,'String','NaN','Position',[x+37 2.3 0])
            if Decision.LaneChange==1
                set(LC_text,'String','向左换道','Position',[x+25 2.3 0])
            elseif Decision.LaneChange==2
                set(LC_text,'String','向右换道','Position',[x+25 2.3 0])
            else
                set(LC_text,'String','NaN','Position',[x+25 2.3 0])
            end
%             set(Title,'String',['TargetLane=' num2str(TargetLaneIndex) 32 32 32 32 'CurrentLane=' num2str(CurrentLaneIndex)]);
            set(T1,'Position',[x+1 2.6 0],'FontSize',12);
            set(T2,'String','目标速度','Position',[x+12 2.6 0],'FontSize',12);
            set(T3,'Position',[x+25 2.6 0],'FontSize',12);
            set(T4,'Position',[x+37 2.6 0],'FontSize',12);
            x=x+ SampleTime;
            axis([x x+50 -0.5 3]);
            drawnow
 %----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        elseif manual==3&&i<=900
            if Decision.LaneChange==1
                set(g,'String','请向左换道','Position',[x+15 ypos 0])
            elseif Decision.LaneChange==2
                set(g,'String','请向右换道','Position',[x+15 ypos 0])
            elseif Decision.SlowDown==1
                set(g,'String',['注意行人，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==2
                set(g,'String',['注意同向车辆，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==3
%                 set(g,'String',['注意对向车，请减速' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+6 ypos 0])
            elseif Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==6
                set(g,'String',['注意前车，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==7
                set(g,'String',['已超速，请减速' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+6 ypos 0])
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
            if Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif (Decision.Wait==1||Decision.Wait==2||Decision.Wait==4)&&speed>0.5
                set(g,'String','请在停止线前停车','Position',[x+13 ypos 0])
            elseif Decision.Wait==3&&speed>0.2
                set(g,'String','请在待转区停车','Position',[x+14 ypos 0])
            end
            drawnow
        elseif manual==3
            if Decision.LaneChange==1
                set(g,'String','请向左换道','Position',[x+15 ypos 0])
            elseif Decision.LaneChange==2
                set(g,'String','请向右换道','Position',[x+15 ypos 0])
            elseif Decision.SlowDown==1
                set(g,'String',['注意行人，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==2
                set(g,'String',['注意同向车辆，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==3
%                 set(g,'String',['注意对向车，请减速' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+6 ypos 0])
            elseif Decision.SlowDown==4
                set(g,'String',['前方路口，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==6
                set(g,'String',['注意前车，请减速至' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+5 ypos 0])
            elseif Decision.SlowDown==7
                set(g,'String',['已超速，请减速' 32 num2str(TargetVelocity) 'km/h'],'Position',[x+6 ypos 0])
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
        if i>784&&usecase==17
            traci.vehicle.setSpeedMode('type4.4', 0);
            traci.vehicle.setSpeed('type4.4',9);
        elseif i>778&&usecase==19
            traci.vehicle.setSpeedMode('type4.4', 0);
            traci.vehicle.setSpeed('type4.4',15);
        elseif i>905&&usecase==20
            traci.vehicle.setSpeedMode('type4.7', 0);
            traci.vehicle.setSpeed('type4.7',9);
        elseif i>970&&usecase==21
            traci.vehicle.setSpeedMode('type5.4', 0);
            traci.vehicle.setSpeed('type5.4',12);
        end
        if i>742&&usecase==18&&SampleTime==0.1
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','12', 2, 98.4, postion(2)+0.34,180);
        elseif i>750&&usecase==18&&SampleTime==0.2
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','12', 2, 98.4, postion(2)+0.7,180);
        elseif i>952&&usecase==22
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 1, 95.2, postion(2)+0.8,180,0);
        elseif i>875&&usecase==23
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 2, 98.4, postion(2)+0.8,180,0);
        elseif i>785&&usecase==24
            postion=traci.vehicle.getPosition('S1');
            traci.vehicle.moveToXY('S1','5', 1, 98.4, postion(2)+1,180,0);
        end
        if usecase==25
            if i>910&&i<950
                postion=traci.vehicle.getPosition('S1');
                traci.vehicle.moveToXY('S1','7', 1, 101.6, postion(2)+2,0,2);
            else
                traci.vehicle.setSpeed('S1',0);
            end
        elseif usecase==26
            if i>900&&i<1000
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
            elseif i==851
                traci.person.setSpeed('p2',3)
            end
        elseif usecase==60
            if i==790
                traci.trafficlights.setPhase('8',2);
            end
        elseif usecase==235
            if i==791
                traci.trafficlights.setPhase('8',2);
            end
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
        elseif usecase==62
            if i>707&&i<900
                traci.vehicle.changeLane('type10.0',2,2);
                traci.vehicle.setSpeed('type10.0',8)
            end
        elseif usecase==63
            if i>715&&i<900
                traci.vehicle.setSpeedMode('type10.0', 0);
                traci.vehicle.setSpeed('type10.0',6)
            end
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
            if i==731
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==801
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            elseif i==851
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==921
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            elseif i==971
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',0)
            elseif i==1041
                traci.vehicle.setSpeedMode('type1.0', 31);
                traci.vehicle.setSpeed('type1.0',10)
            end
        elseif usecase==129
            if i==730
                traci.person.setSpeed('p5.0',0)
            end
        elseif usecase==232
            if i==731
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
            if i==1100
                traci.vehicle.setSpeed('type44.0',0);
            elseif i==1200
                traci.vehicle.setSpeed('type44.0',30/3.6);
            end
        elseif usecase==180
            Postion=traci.vehicle.getPosition('S0');
            if i==750
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)-2, Postion(2),0,2);
            elseif i==800
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)+2, Postion(2),0,2);
            elseif i==850
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)-1.5, Postion(2),0,2);
            end
        elseif usecase==182
            Postion=traci.vehicle.getPosition('S0');
            if i==780
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)-2, Postion(2),0,2);
            elseif i==1030
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)+2, Postion(2),0,2);
            end
        elseif usecase==183
            Postion=traci.vehicle.getPosition('S0');
            if i==701
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1), Postion(2),11,2);
            elseif i==800
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1), Postion(2),349,2);
            end
        elseif usecase==184
            Postion=traci.vehicle.getPosition('S0');
            if i==701
                traci.vehicle.moveToXY('S0','M0', 2, Postion(1)-2, Postion(2),330,2);
%             elseif i==800
%                 traci.vehicle.moveToXY('S0','M0', 2, Postion(1), Postion(2),20,2);
            end
        end
%         if Draw==1
%             T1=etime(t2,t1);
%             T2=etime(t3,t2);
%             T3=etime(t4,t3);
%             T4=etime(t5,t1);
%             T5=etime(t6,t5);
%             T6=etime(t7,t6);
%             T7=etime(t8,t7);
%             T8=etime(t9,t8);
%             T9=etime(t10,t9);
%             T10=etime(t2,t10);
%             w(1,i)=double(T1);
%             w(2,i)=double(T2);
%             w(3,i)=double(T3);
%             w(4,i)=double(T4);
%             w(5,i)=double(T5);
%             w(6,i)=double(T6);
%             w(7,i)=double(T7);
%             w(8,i)=double(T8);
%             w(9,i)=double(T9);
%             w(10,i)=double(T10);            
%             set(p(1),'XData',t,'YData',w(1,:))
%             set(p(2),'XData',t,'YData',w(2,:))
%             set(p(3),'XData',t,'YData',w(3,:))
%             set(p(4),'XData',t,'YData',w(4,:))
%             set(p(5),'XData',t,'YData',w(5,:))
%             set(p(6),'XData',t,'YData',w(6,:))
%             set(p(7),'XData',t,'YData',w(7,:))
%             set(p(8),'XData',t,'YData',w(8,:))
%             set(p(9),'XData',t,'YData',w(9,:))
%             set(p(10),'XData',t,'YData',w(10,:))
%             x=x+0.1;
%             axis([x x+50 -0.2 0.5]);
%             drawnow
%         end
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
     elseif usecase==174%异常停车
        if i>500
            PositionCars=traci.vehicle.getLanePosition('type2.0');
            if PositionCars>95
                traci.vehicle.setSpeed('type2.0',0.09);
                traci.vehicle.changeLane('type2.0',1,2);
            end
        end     
      elseif usecase==175%异常低速
        if i>500
            PositionCars=traci.vehicle.getLanePosition('type2.0');
            if PositionCars>55
                traci.vehicle.setSpeed('type2.0',3);
                traci.vehicle.changeLane('type2.0',1,2);
            end
        end
    elseif usecase==176
        if i==720
                traci.vehicle.setSpeed('type2.0',0.09);
        end 
         if i>=750&&i<=1000
                Angle=traci.vehicle.getAngle('type2.0');
                postion=traci.vehicle.getPosition('type2.0');
                traci.vehicle.moveToXY('type2.0','E12', 2, postion(1), postion(2)-0.4,180,2);   
         end
      elseif usecase==177%超视距前碰撞预警
          if i>650&&i<1300
              vehicleLane1=traci.vehicle.getLaneIndex('type2.0');
              vehicleLane2=traci.vehicle.getLaneIndex('type3.0');
              traci.vehicle.changeLane('type2.0',vehicleLane1,2);
              traci.vehicle.changeLane('type3.0',vehicleLane2,2);
              if i==981
                  traci.vehicle.setSpeed('type2.0',3);
                  traci.vehicle.setMinGap('type3.0',0);
              end
              if i==1021
                  traci.vehicle.setSpeed('type2.0',0);
              end
          end
      elseif usecase==178%紧急制动
          if i>650&&i<1300
              vehicleLane1=traci.vehicle.getLaneIndex('type2.0');
              vehicleLane2=traci.vehicle.getLaneIndex('type3.0');
              traci.vehicle.changeLane('type2.0',vehicleLane1,2);
              traci.vehicle.changeLane('type3.0',vehicleLane2,2);
              if i==981
                  traci.vehicle.setSpeed('type2.0',3);
                  traci.vehicle.setMinGap('type3.0',0);
              end
              if i==1021
                  traci.vehicle.changeLane('type3.0',vehicleLane2+1,2);
                  traci.vehicle.setSpeed('type2.0',0);
              end
          end
          
%         if i>500
%             PositionCars=traci.vehicle.getLanePosition('type2.0');
%             if PositionCars>55
%                 traci.vehicle.setSpeed('type2.0',3);
%                 traci.vehicle.changeLane('type2.0',1,2);
%             end
%         end
%         if i>550
%             PositionCars=traci.vehicle.getLanePosition('type3.0');
%             if PositionCars>55
%                 traci.vehicle.setSpeed('type2.0',3);
%                 traci.vehicle.changeLane('type2.0',1,2);
%             end
%         end
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
function [tv]=Frontcollision(Af,Sd,Vf,L_veh,speed)
a_set=-2.5;
if Af>=a_set
    tv=(Sd-L_veh-5+(speed^2-Vf^2)/(2*a_set))/speed;
else
    tv=(Sd-L_veh-5+(speed^2)/(2*a_set)-Vf^2/(2*a_set))/speed;
end
end
function [pos_x,pos_y,pos_yaw]=frenet2XY(pos_s,pos_l,pos_psi,nodelist_s,laneshape)
node_index=find(nodelist_s>pos_s,1);
if node_index>=length(laneshape)
    node_index=length(laneshape);
end
snode_x=laneshape{1,node_index-1}(1,1);
snode_y=laneshape{1,node_index-1}(1,2);
enode_x=laneshape{1,node_index}(1,1);
enode_y=laneshape{1,node_index}(1,2);
theta=atan2((enode_y-snode_y),(enode_x-snode_x))*180/pi;
pos_yaw=pos_psi-theta;
pos_s=pos_s-nodelist_s(node_index-1);
pos_x=pos_s*cosd(theta)-pos_l*sind(theta)+snode_x;
pos_y=pos_s*sind(theta)+pos_l*cosd(theta)+snode_y;
end
function [pos_s,pos_l,pos_psi]=XY2frenet(x,y,yaw,nodelist_s,laneshape)
for node_index=2:length(laneshape)
    snode_x=laneshape{1,node_index-1}(1,1);
    snode_y=laneshape{1,node_index-1}(1,2);
    enode_x=laneshape{1,node_index}(1,1);
    enode_y=laneshape{1,node_index}(1,2);
    theta=atan2((enode_y-snode_y),(enode_x-snode_x))*180/pi;
    pos_psi=mod(yaw+theta,360);
    pos_s=(x-snode_x)*cosd(theta)+(y-snode_y)*sind(theta);
    pos_s=pos_s+nodelist_s(node_index-1);
    pos_l=-(x-snode_x)*sind(theta)+(y-snode_y)*cosd(theta);
    if pos_s<=nodelist_s(node_index)||node_index==length(laneshape)
        break
    end
end
end