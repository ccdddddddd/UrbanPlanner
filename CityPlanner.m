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
PrePedestrianActive=0;
AEBActive=0;

% Launch SUMO in server mode and initialize the TraCI connection
traci.start('sumo-gui -c ./CityAvoidPed.sumocfg --start');

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
        
        % 确定与人行横道距离
         if strcmp(current_road_ID,'1')
            % d_veh2cross=max([80-currentLanePosition-widthCrossing 0]);
            d_veh2cross=max([95.3-postion_veh(2) 0]);
        end
        % 确定行人位置、速度
        if strcmp(current_road_ID,'1')
            % PersonIDs=[traci.edge.getLastStepPersonIDs('3') traci.edge.getLastStepPersonIDs('4') traci.edge.getLastStepPersonIDs(':1_w1')];
            PersonIDs=traci.edge.getLastStepPersonIDs(':1_w1');
            personPositions=zeros(length(PersonIDs),1);
            personSpeeds=zeros(length(PersonIDs),1);
            for id=1:length(PersonIDs)
                personPosition=traci.person.getPosition(PersonIDs{id});
                personPositions(id)=abs(personPosition(1)-1.6);
                personSpeed=traci.person.getSpeed(PersonIDs{id});
                personAngle=90-traci.person.getAngle(PersonIDs{id});
                personSpeeds(id)=-personSpeed*cosd(personAngle)*sign(personPosition(1)-1.6);
            end
            if ~isempty(PersonIDs)
                v_ped=max(personSpeeds);
                s_ped=min(personPositions);
            end
        end
        % 确定与待转区距离
        % 98.4,-1.6 cross1
        % 95.2,0.65 cross2
        %
        % 101.60,-10.40
        % (( 98.4-postion_veh(1)).^2+(-1.6-postion_veh(2)).^2).^0.5
        % (( 98.4-95.2).^2+(-1.6-0.65).^2).^0.5

        % 搜寻对向车

        % 搜寻前车
        [aID, adist]=traci.vehicle.getLeader('S0',100);
        if isempty(aID)==0
            CurrentLaneFrontVel=traci.vehicle.getSpeed(aID);
            CurrentLaneFrontDis=adist+traci.vehicle.getMinGap('S0')+l_veh;
        end
        % 搜寻左右车
        
        % 搜寻不可通行车道
% 速度规划         
        % if (strcmp(current_road_ID,'5') || strcmp(current_road_ID,':5_1'))  && pos_l>103.2
        if strcmp(current_road_ID,'1')
            LaneChangeActive=0;
            PedestrianActive=1;
            TrafficLightActive=0;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
                dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
                PrePedestrianActive,AEBActive]=...,
                UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
                CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
                pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
                w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
                d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,PrePedestrianActive,AEBActive,...,
                LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex);
            
            traci.vehicle.setSpeed('S0',traj_vs(2));
            %     traci.vehicle.changeLane('S0',CurrentLaneIndex,2);
            % pause(0.05);
        else
            LaneChangeActive=0;
            PedestrianActive=0;
            TrafficLightActive=0;
            VehicleCrossingActive=0;
            VehicleOncomingActive=0;
            [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,...,
                dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,CurrentTargetLaneIndex,...,
                PrePedestrianActive,AEBActive]=...,
                UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,...,
                CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,speed,...,
                pos_s,pos_l_CurrentLane,CurrentLaneIndex,CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,d_veh2converge,d_veh2stopline,w_lane_left,w_lane_right,dec_ped,d_veh2cross,...,
                w_cross,wait_ped,s_ped,v_ped,dec_fol_TrafficLight,dec_bre_TrafficLight,wait_TrafficLight,greenLight,time2nextSwitch,v_max,dec_fol_AvoidVehicle,dec_bre_AvoidVehicle,wait_AvoidVehicle,dec_avoidOncomingVehicle,...,
                d_veh2waitingArea,wait_avoidOncomingVehicle,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,PrePedestrianActive,AEBActive,...,
                LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive,VehicleOncomingActive,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc_RePlan,pos_l,NumOfLanes,LanesWithFail,CurrentTargetLaneIndex);
            
            traci.vehicle.setSpeed('S0',traj_vs(2));
            
        end
        pause(0.05);

        % 用例 10-1 "if strcmp(current_road_ID,'1') && i>762 LaneChangeActive=1;"
        % if i>730 && i<762
        %     traci.vehicle.setSpeed('S0',0);
        % end
        % 
        % 用例 10-2 "if strcmp(current_road_ID,'1') && i>761 LaneChangeActive=1;"
        % if i>730 && i<761
        %     traci.vehicle.setSpeed('S0',0);
        % end
%         if i>740 && s_ped<800
%             traci.person.setSpeed('p1',0)
%         end
        if i>780
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
