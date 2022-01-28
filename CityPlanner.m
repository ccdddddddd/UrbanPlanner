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
%*******


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
TrafficLightOffset=70; % 路口8的
dec_ped=0;
wait_ped=0;
dec_avoidOncomingVehicle=0;
wait_avoidOncomingVehicle=0;
CurrentTargetLaneIndex=0;
% PreVehicleCrossingActive=0;
PrePedestrianActive=0;
AEBActive=0;

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

        traci.vehicle.setSpeedMode('S0', 0)
% set view of sumo gui
        traci.gui.trackVehicle('View #0','S0');
        traci.gui.setZoom('View #0', 600);
        yaw=traci.vehicle.getAngle('S0');
        lane=traci.vehicle.getLaneIndex('S0') ;
        speed=max([0 traci.vehicle.getSpeed('S0')]) ;
        current_road_ID=traci.vehicle.getRoadID('S0');
        current_lane_ID=traci.vehicle.getLaneID('S0');
        postion_veh=traci.vehicle.getPosition('S0');
        pos_s=0;
        pos_l=postion_veh(1);
        pos_l_CurrentLane=0;
        % NumOfLanes=6;
        CurrentLaneIndex=traci.vehicle.getLaneIndex('S0');
        TargetLaneIndex=CurrentLaneIndex;
        route = traci.vehicle.getRoute('S0');
        
% 本车frenet坐标

% 搜寻前车、后车、与路口距离、行人位置、与人行横道距离,对向车、与待转区距离
        currentLanePosition=traci.vehicle.getLanePosition('S0'); 
        
        % 确定与路口的距离
        if strcmp(current_road_ID,'7')
            d_veh2int=max([85.6-currentLanePosition 0]);
        elseif strcmp(current_road_ID,':8_8') || strcmp(current_road_ID,':8_9')
            d_veh2int=-currentLanePosition;
        end

        % 确定与人行横道距离

        % 确定行人位置、速度
        
        % 确定与待转区距离
        if strcmp(current_road_ID,'7')
            d_veh2cross1=max([9.3638+85.6-currentLanePosition 0]);
            d_veh2cross2=d_veh2cross1+3.9118;
            % d_veh2waitingArea=max([d_veh2cross1-5 0]);
            d_veh2waitingArea=d_veh2cross1-3;
        elseif strcmp(current_road_ID,':8_11') || strcmp(current_road_ID,':8_18')
            d_veh2cross1=sign(postion_veh(1)-98.4)*(( 98.4-postion_veh(1)).^2+(-1.6-postion_veh(2)).^2).^0.5;
            d_veh2cross2=max([d_veh2cross1+3.9118 0]);
            d_veh2cross1=max([d_veh2cross1 0]);
            % d_veh2waitingArea=max([d_veh2cross1-5 0]);
            d_veh2waitingArea=d_veh2cross1-3;
        end

        % 搜寻对向车
        if strcmp(current_road_ID,'7') || strcmp(current_road_ID,':8_11') || strcmp(current_road_ID,':8_18')
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('9_1') traci.lane.getLastStepVehicleIDs(':8_1_1')];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getPosition(oncomingVehiclesIDs{id});
                oncomingVehiclesPositions(id)=oncomingVehiclePosition(2)--1.6;
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>0);            
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<=0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>0)));
                    s_veh1=oncomingVehiclesPositions(veh1ID);
                    v_veh1=oncomingVehiclesSpeeds(veh1ID);
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=find(oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0)));
                    s_veh1apostrophe1=oncomingVehiclesPositions(veh1apostrophe1ID);
                end
            end        
        end
        if strcmp(current_road_ID,'7') || strcmp(current_road_ID,':8_11') || strcmp(current_road_ID,':8_18')
            oncomingVehiclesIDs=[traci.lane.getLastStepVehicleIDs('9_0') traci.lane.getLastStepVehicleIDs(':8_1_0')];
            oncomingVehiclesPositions=zeros(length(oncomingVehiclesIDs),1);
            oncomingVehiclesSpeeds=zeros(length(oncomingVehiclesIDs),1);
            for id=1:length(oncomingVehiclesIDs)
                oncomingVehiclePosition=traci.vehicle.getPosition(oncomingVehiclesIDs{id});
                oncomingVehiclesPositions(id)=oncomingVehiclePosition(2)-0.65;
                oncomingVehiclesSpeeds(id)=traci.vehicle.getSpeed(oncomingVehiclesIDs{id});
            end
            if ~isempty(oncomingVehiclesIDs)
                veh1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions>0);            
                veh1apostrophe1IDs=oncomingVehiclesPositions(oncomingVehiclesPositions<=0);
                if isempty(veh1IDs)==0
                    veh1ID=find(oncomingVehiclesPositions==min(oncomingVehiclesPositions(oncomingVehiclesPositions>0)));
                    s_veh2=oncomingVehiclesPositions(veh1ID);
                    v_veh2=oncomingVehiclesSpeeds(veh1ID);
                end
                if isempty(veh1apostrophe1IDs)==0
                    veh1apostrophe1ID=find(oncomingVehiclesPositions==max(oncomingVehiclesPositions(oncomingVehiclesPositions<0)));
                    s_veh1apostrophe2=oncomingVehiclesPositions(veh1apostrophe1ID);
                end
            end        
        end

        % 搜寻前车
        [aID, adist]=traci.vehicle.getLeader('S0',100);
        if isempty(aID)==0
            CurrentLaneFrontVel=traci.vehicle.getSpeed(aID);
            CurrentLaneFrontDis=adist+traci.vehicle.getMinGap('S0')+l_veh;
        end
        % 搜寻左右车
        
        % 搜寻匝道汇入时主路上车辆

        % 搜寻不可通行车道
% 速度规划         
        if strcmp(current_road_ID,'7')
            % 进入路口8前直行

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
            if strcmp(route{length(route)},'8')
                LaneChangeActive=0;
                PedestrianActive=0;
                TrafficLightActive=1;
                VehicleCrossingActive=0;
                VehicleOncomingActive=1;
            else
                LaneChangeActive=0;
                PedestrianActive=0;
                TrafficLightActive=1;
                VehicleCrossingActive=0;
                VehicleOncomingActive=0;
            end
            [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
                dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
                AEBActive]=...,
                UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
                CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
                pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
                w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
                d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,DurationLaneChange_RePlan,...,
                LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex,AEBActive,...,
                LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive);
            
            traci.vehicle.setSpeed('S0',traj_vs(2));

        elseif strcmp(current_road_ID,':8_11') || strcmp(current_road_ID,':8_18')
            % 路口8中左转
            
            % 需要给出：
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=0;
            VehicleCrossingActive=0;
            VehicleOncomingActive=1;
            [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
                dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
                AEBActive]=...,
                UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
                CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
                pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
                w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
                d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,DurationLaneChange_RePlan,...,
                LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex,AEBActive,...,
                LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive);
%             if i>905 && i<920
%                 traci.vehicle.setSpeedMode('type3.15', 0);
%                 traci.vehicle.setSpeed('type3.15',20);
%             end
%             if i==921
%                 traci.vehicle.setSpeedMode('type3.15', 31); % 恢复考虑碰撞的速度模式
%             end
            
            traci.vehicle.setSpeed('S0',traj_vs(2));
            pause(0.05);
        elseif strcmp(current_road_ID,':8_9')
            % 路口8中直行

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
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=0;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
                dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
                AEBActive]=...,
                UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
                CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
                pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
                w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
                d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,DurationLaneChange_RePlan,...,
                LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex,AEBActive,...,
                LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive);
            
            traci.vehicle.setSpeed('S0',traj_vs(2));
            pause(0.1);
        end
        

        % 用例 10-1 "if strcmp(current_road_ID,'1') && i>762 LaneChangeActive=1;"
        % 
        % 用例 10-2 "if strcmp(current_road_ID,'1') && i>761 LaneChangeActive=1;"
        % if i>730 && i<761
        %     traci.vehicle.setSpeed('S0',0);
        % end
%         if i>740 && s_ped<800
%             traci.person.setSpeed('p1',0)
%         end
        if i==813
            traci.trafficlights.setPhase('8',2);
        end

        if i>850
            
%         if wait>0
%             a_soll
%             pause(0.01);
%             d_ist
%             v_soll
%             wait
%             a_soll
%             a_soll*0.1+speed
%             11111111111111111111111111111111111
%             s_a
%             v_a
%             s_b
%             v_b
%             wait
%             s_c
%             v_c
        end

    end
end
traci.close();
