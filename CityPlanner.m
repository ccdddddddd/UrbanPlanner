clear;
close('all');
clc;

wait=0;

% 设置参数
import traci.constants;
a_min_com=-1.5;
a_max=2.5;
a_min=-3;
a_dec=0;
v_max=50/3.6;
v_max_int=30/3.6;
l_veh=5;
w_weh=1.8;
t_re=1.5;
GapIndex=3;
widthCrossing=1;
NumOfLanes=1;
TurningRadius=13;
% 全局变量
CountLaneChange=0;
DurationLaneChange=0;
LaneChangePath=zeros([6/0.05 6]);
DurationLaneChange_RePlan=0;
LaneChangePath_RePlan=zeros([6/0.05 6]);
t_lc_RePlan=0;
w_lane_left=3.2;
w_lane_right=3.2;
t_lc_traj=2;
dec_fol_TrafficLight=0;
dec_bre_TrafficLight=0;
wait_TrafficLight=0;
dec_fol_AvoidVehicle=0;
dec_bre_AvoidVehicle=0;
wait_AvoidVehicle=0;
d_fol=0;
d_bre=0;
dec_bre=0;
dec_fol=0;
TrafficLightOffset=30; % 路口8的
TrafficLightOffset2=0; % 路口11的
TrafficLightOffset3=0; % 路口gneJ0的
dec_ped=0;
wait_ped=0;
dec_avoidOncomingVehicle=0;
wait_avoidOncomingVehicle=0;
CurrentTargetLaneIndex=0;
AEBActive=0;
TurnAroundActive=0;
s_circle1=0;
l_circle1=0;
s_circle2=0;
l_circle2=0;
dec_trunAround=0;
wait_turnAround=0;
TypeOfTurnAround=0;
TargetLaneIndexOpposite=0;
PosCircle1=[];
PosCircle2=[];
PosCircle3=[];
pos_start=[];
pos_mid1=[];
pos_mid2=[];
pos_mid1_rear=[];
pos_mid2_rear=[];
pos_end=[];
LaneCenterline=[];
TargetGear=0;
CurrentGear=0;
TurnAroundState=0;
% Launch SUMO in server mode and initialize the TraCI connection
traci.start('sumo-gui -c ./City.sumocfg --start');
SIM_STEPS = [1 3560];
beginTime = SIM_STEPS(1);
duration =  SIM_STEPS(2);
endTime =  SIM_STEPS(1) +  SIM_STEPS(2) - 1;
% phase = zeros(1,duration);
% traci.vehicle.setColor('S0',[0 255 255 200]);
for i = 1: duration
    traci.simulation.step();
    if i>701
        CurrentLaneFrontDis=200;
        CurrentLaneFrontVel=20;
        LeftLaneBehindDis=-200;
        LeftLaneBehindVel=20;
        LeftLaneFrontDis=200;
        LeftLaneFrontVel=20;
        RightLaneBehindDis=-200;
        RightLaneBehindVel=-20;
        RightLaneFrontDis=200;
        RightLaneFrontVel=20;
        CurrentLaneFrontDisAvoidVehicle=200;
        CurrentLaneFrontVelAvoidVehicle=20;
        TargetLaneBehindDisAvoidVehicle=-200;
        TargetLaneBehindVelAvoidVehicle=20;
        TargetLaneFrontDisAvoidVehicle=200;
        TargetLaneFrontVelAvoidVehicle=20;
        d_veh2int=200;
        d_veh2converge=200;
        d_veh2stopline=200;
        s_ped=100;
        v_ped=0;
        d_veh2cross=200;
        greenLight=1;
        time2nextSwitch=100;
        LanesWithFail=[];
        w_cross=5;
        d_veh2waitingArea=200;
        s_veh1=200;
        v_veh1=0;
        d_veh2cross1=0;
        s_veh1apostrophe1=-200;
        s_veh2=200;
        v_veh2=0;
        d_veh2cross2=0;
        s_veh1apostrophe2=-200;
        s_veh3=200;
        v_veh3=0;
        d_veh2cross3=0;
        s_veh1apostrophe3=-200;
        %         IndexOfLaneOppositeCarFront=zeros([6,1]);
        %         SpeedOppositeCarFront=zeros([6,1]);
        %         PosSOppositeCarFront=zeros([6,1]);
        %         PosSOppositeCarFront=PosSOppositeCarFront+200;
        %         PosSOppositeCarRear=zeros([6,1]);
        %         PosSOppositeCarRear=PosSOppositeCarRear-200;
        IndexOfLaneOppositeCar=zeros([20,1]);
        SpeedOppositeCar=zeros([20,1]);
        PosSOppositeCar=zeros([20,1]);
        PosSOppositeCar=PosSOppositeCar-200;
        IndexOfLaneCodirectCar=zeros([10,1]);
        SpeedCodirectCar=zeros([10,1])-1;
        PosSCodirectCar=zeros([10,1]);   
        traci.vehicle.setSpeedMode('S0', 0)
        % set view of sumo gui
%         traci.gui.trackVehicle('View #0','S0');
%         traci.gui.setZoom('View #0', 600);
        traci.gui.setBoundary('View #0',-60,20,20,190);
        yaw=traci.vehicle.getAngle('S0');
        lane=traci.vehicle.getLaneIndex('S0') ;
        speed=max([0 traci.vehicle.getSpeed('S0')]) ;
        current_road_ID=traci.vehicle.getRoadID('S0');
        current_lane_ID=traci.vehicle.getLaneID('S0');
        postion_veh=traci.vehicle.getPosition('S0');
        % NumOfLanes=6;
        CurrentLaneIndex=traci.vehicle.getLaneIndex('S0');
        route = traci.vehicle.getRoute('S0');
        % 本车frenet坐标
        pos_s=postion_veh(2);
%         pos_l=101.6-postion_veh(1);
        pos_l=101.6-postion_veh(1);
        pos_l_CurrentLane=0;
        TargetLaneIndex=CurrentLaneIndex;
        % 搜寻前车、后车、与路口距离、行人位置、与人行横道距离,对向车、与待转区距离
        currentLanePosition=traci.vehicle.getLanePosition('S0');
        % 确定对向道路信息
        if strcmp(current_road_ID,'7')
            NumOfLanesOpposite=3;
            WidthOfLanesOpposite=[3.2,3.2,3.2,0,0,0];
            WidthOfGap=0;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=-6.4;
        elseif strcmp(current_road_ID,'10')
            NumOfLanesOpposite=2;
            WidthOfLanesOpposite=[3.2,3.2,0,0,0,0];
            WidthOfGap=0;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=146.8;
        elseif strcmp(current_road_ID,'gneE0')
            NumOfLanesOpposite=1;
            WidthOfLanesOpposite=[3.2,0,0,0,0,0];
            WidthOfGap=3.2;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=246.8;
        else
            NumOfLanesOpposite=0;
            WidthOfLanesOpposite=[3.2,3.2,3.2,0,0,0];
            WidthOfGap=0;
            WidthOfLaneCurrent=3.2;
            s_turnaround_border=0;
        end
        % 确定与路口的距离
        if strcmp(current_road_ID,'7')%第一个路口
            d_veh2int=-10.4-pos_s;
        elseif strcmp(current_road_ID,'10')
            d_veh2int=142.8-pos_s;
        elseif strcmp(current_road_ID,'gneE0')
            d_veh2int=242.8-pos_s;
        end
        if d_veh2int<0
            d_veh2int=200;
        end
        % 确定与人行横道距离
        % 确定行人位置、速度
        % 确定与待转区距离 
        %--------------------------------------------------------------------------------------------------------------------------------------------------
        % 搜寻掉头对向车
        if pos_s>-50&&pos_s<10%第一个路口
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
            end
        elseif pos_s>100&&pos_s<160%第二个路口
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
            end
            
        elseif pos_s>200&&pos_s<260%第三个路口
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
            end
        end
        %搜寻掉头前车道环境车
        if pos_s>-50&&pos_s<10%第一个路口
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
            end
        elseif pos_s>100&&pos_s<160%第二个路口
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
            end
        elseif pos_s>200&&pos_s<260%第三个路口
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
            end
        end
        % 搜寻对向车
        % 搜寻前车
        [aID, adist]=traci.vehicle.getLeader('S0',100);
        if isempty(aID)==0
            CurrentLaneFrontVel=traci.vehicle.getSpeed(aID);
            CurrentLaneFrontDis=adist+traci.vehicle.getMinGap('S0')+l_veh;
        end
        % 搜寻左右车
        % 搜寻匝道汇入时主路上车辆
        % 搜寻不可通行车道      
        % 档位变换
        CurrentGear=TargetGear;
        % 速度规划
        if strcmp(current_road_ID,'7') && strcmp(route{length(route)-1},'7')
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
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=0;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            TurnAroundActive=1;
        elseif strcmp(current_road_ID,'10')&& strcmp(route{length(route)-1},'10')
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
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=1;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            TurnAroundActive=1;
        elseif strcmp(current_road_ID,'gneE0')&& strcmp(route{length(route)-1},'gneE0')
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
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=1;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            TurnAroundActive=1;
            
        else
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=1;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
        end
        [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
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
            LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,TurnAroundActive);
        if a_soll~=100
            % traci.vehicle.setSpeed('S0',traj_vs(2));
            traci.vehicle.setSpeed('S0',sqrt((traj_vs(2)).^2+(traj_vl(2)).^2));
        else
            traci.vehicle.moveToXY('S0','12', 2, 101.6-traj_l(2), traj_s(2),traj_psi(2)-90,2);
            traci.vehicle.setSpeed('S0',sqrt((traj_vs(2)).^2+(traj_vl(2)).^2));
        end
        %         if pos_s>140
        %             pause(0.02);
        %         end
        if i<=1500
            if i>1000&&strcmp(traci.vehicle.getRoadID('type4.7'),':8_1')
                traci.vehicle.changeLane('type4.7', 0,0);
            end
            if i>1000&&strcmp(traci.vehicle.getRoadID('type4.7'),'5')
                traci.vehicle.changeLane('type4.7', 1,0);
            end
            if i>1000&&strcmp(traci.vehicle.getRoadID('type4.10'),'9')
                traci.vehicle.changeLane('type4.10', 0,1);
            end
            if i>1000&&strcmp(traci.vehicle.getRoadID('type4.10'),':8_1')
                traci.vehicle.changeLane('type4.10', 0,1);
            end
            if i>1000&&strcmp(traci.vehicle.getRoadID('type4.10'),'5')
                traci.vehicle.changeLane('type4.10', 1,1);
            end
            if i>1400
                traci.vehicle.setSpeedMode('type4.7', 0);
                traci.vehicle.setSpeed('type4.7',8);
            end
            if i>1410
                traci.vehicle.setSpeedMode('type4.10', 0);
                traci.vehicle.setSpeed('type4.10',8);
            end
        end
%-----------------------------------------------------------------------------------------------------------------------------------------------------------
%仿真
%                 if i==750
%                     i
%                 end
%         if i>788
%             traci.vehicle.setSpeedMode('type4.4', 0);
%             traci.vehicle.setSpeed('type4.4',-5);
%         end
%用例14-22
%         if i>1000
%             postion=traci.vehicle.getPosition('S1');
%             traci.vehicle.moveToXY('S1','5', 1, 98.4, postion(2)+0.8,180,0);
%         end
%用例14-23
%         if i>880
%             postion=traci.vehicle.getPosition('S1');
%             traci.vehicle.moveToXY('S1','5', 2, 98.4, postion(2)+0.8,180,0);
%         end
%用例14-24
%         if i>820
%             postion=traci.vehicle.getPosition('S1');
%             traci.vehicle.moveToXY('S1','5', 1, 98.4, postion(2)+1,180,0);
%         end
%用例14-25
%         if i>910&&i<1000
%             postion=traci.vehicle.getPosition('S1');
%             traci.vehicle.moveToXY('S1','7', 1, 101.6, postion(2)+2,0,2);
%         else
%             traci.vehicle.setSpeed('S1',0);
%         end
%用例14-26
%         if i>910&&i<1000
%             postion=traci.vehicle.getPosition('S1');
%             traci.vehicle.moveToXY('S1','7', 1, 101.6, postion(2)-1,0,2);
%         else
% %             postion=traci.vehicle.getPosition('S1');
% %             traci.vehicle.moveToXY('S1','5', 1, 101.6, postion(2),0,2);
%         end
%         if i>1024
%             traci.vehicle.setSpeedMode('type5.4', 0);
%             traci.vehicle.setSpeed('type5.4',12);
%         end
        % 用例 10-1 "if strcmp(current_road_ID,'1') && i>762 LaneChangeActive=1;"
        %
        % 用例 10-2 "if strcmp(current_road_ID,'1') && i>761 LaneChangeActive=1;"
        % if i>730 && i<761
        %     traci.vehicle.setSpeed('S0',0);
        % end
        %         if i>800&&i<833
        %             Veh_speed(i-800)=speed;
        %         end
    end
end
traci.close();
 %-----------------------------------------------------------------------------------------------------------------------------------------------------------
%搜直道车信息
function [VehicleLaneSSpeed]=VehicleInform1(VehiclesIDLane,s_turnaround_border,pos_start,Lane)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),2);
    for id=1:length(VehiclesIDLane)
        Position=traci.vehicle.getPosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=Position(2);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
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
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),3);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:3)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end
function [VehicleLaneSSpeed]=VehicleInform2(VehiclesIDLane,s_turnaround_border,pos_start,Lane,LaneRefPos)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),2);
    for id=1:length(VehiclesIDLane)
        VehiclesLane(id,1)=traci.vehicle.getLanePosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=LaneRefPos+VehiclesLane(id,1);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
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
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),3);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:3)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end
function [VehicleLaneSSpeed]=VehicleInform3(VehiclesIDLane,s_turnaround_border,pos_start,Lane,LaneRefPos,Lanelength)
if isempty(VehiclesIDLane)==0
    VehiclesLane=zeros(length(VehiclesIDLane),2);
    for id=1:length(VehiclesIDLane)
        VehiclesLane(id,1)=traci.vehicle.getLanePosition(VehiclesIDLane{id});
        VehiclesLane(id,1)=LaneRefPos+Lanelength-VehiclesLane(id,1);
        VehiclesLane(id,2)=traci.vehicle.getSpeed(VehiclesIDLane{id});
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
    VehicleLaneSSpeed=zeros(length(Vehicles2Lane(:,1)),3);
    if isempty(Vehicles2Lane)==0
        VehicleLaneSSpeed(:,1)=Lane;
        VehicleLaneSSpeed(:,2:3)=Vehicles2Lane;
    end
else
    VehicleLaneSSpeed=[];
end
end