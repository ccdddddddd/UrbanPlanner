function [AEBActive,GlobVars]=AEBDecision(AEBActive,speed,d_veh2stopline_ped,d_veh2stopline,...,
    d_veh2waitingArea,s_veh,v_veh,d_veh2conflict,s_veh1apostrophe,d_veh2int,greenLight,GlobVars,CalibrationVars,Parameters)
%--------------------------------------------------------------------------------------------------------------------------------------------------------
wait_TrafficLight=GlobVars.SpeedPlanTrafficLight.wait_TrafficLight;
wait_ped=GlobVars.SpeedPlanAvoidPedestrian.wait_ped;
wait_AvoidVehicle=GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle;
wait_avoidOncomingVehicle=GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;
% 紧急停车决策
l_veh=Parameters.l_veh;
w_veh=Parameters.w_veh;
v_max_int=CalibrationVars.SpeedPlanTrafficLight.v_max_int;%30/3.6;
D_safe=CalibrationVars.SpeedPlanAvoidOncomingVehicle.D_safe;%2
a_max_com=1.5;
if AEBActive==0
    % if PrePedestrianActive==1 && PedestrianActive==0 && wait_ped==1 && speed>0
    if (0-speed.^2)/(2*CalibrationVars.ACC.a_min)>=d_veh2stopline_ped && wait_ped==1 && speed>0 % 避让行人决策 → AEB
        % 判断是否碰撞行人
        AEBActive=int16(1);
        wait_ped=int16(0);
    end
    if d_veh2stopline<=0 && wait_AvoidVehicle==1 && speed>0 && AEBActive==0 % 避让同向车辆决策 → AEB
        AEBActive=int16(2);
        wait_AvoidVehicle=int16(0);
    end
    % if d_veh2int<=0 && d_veh2int>=-0.5*l_veh && greenLight==0 && speed>0 && AEBActive==0 % 红绿灯通行决策 → AEB
    if d_veh2int<=0 && (0-speed.^2)/(2*-4)-d_veh2int<=10 && greenLight==0 && speed>0 && AEBActive==0 % 红绿灯通行决策 → AEB
        AEBActive=int16(4);
        wait_TrafficLight=int16(0);
    end
    if d_veh2waitingArea<=0 && speed>0 && AEBActive==0 % 避让对向车辆决策 → AEB
        %         if wait_avoidOncomingVehicle==1
        %             AEBActive=3;
        %             wait_avoidOncomingVehicle=0;
        %         else
        for i=1:1:6
            if d_veh2conflict(i)==0
                s_veh(i)=200;
                v_veh(i)=0;
                s_veh1apostrophe(i)=-200;
            end
        end
        %             d_veh1=max([(d_veh2cross1+l_veh)/max([speed 0.00001])*v_veh1+0.5*w_veh+l_veh 0]);
        %             d_veh2=max([(d_veh2cross2+l_veh)/max([speed 0.00001])*v_veh2+0.5*w_veh+l_veh 0]);
        %             d_veh3=max([(d_veh2cross3+l_veh)/max([speed 0.00001])*v_veh3+0.5*w_veh+l_veh 0]);
        %             if s_veh1<=d_veh1 || s_veh1apostrophe1>-l_veh || s_veh2<=d_veh2 || s_veh1apostrophe2>-l_veh || s_veh3<=d_veh3 || s_veh1apostrophe3>-l_veh
        %                 AEBActive=3;
        %             end
        timeGap=zeros(1,6);
        s_max=zeros(1,6);
        for i=1:1:6
            timeGap(i)=max([0 (s_veh(i)-0.5*w_veh-D_safe)/max([v_veh(i) 0.00001])]);
            s_max(i)=0.5*(min([speed+a_max_com*timeGap(i) v_max_int])+speed)*timeGap(i);
        end
        for i=1:1:6
            if ~(s_max(i)>d_veh2conflict(i)+l_veh && s_veh1apostrophe(i)<-l_veh)
                AEBActive=int16(3);
                break;
            end
        end
        %         end
    end
end
GlobVars.SpeedPlanTrafficLight.wait_TrafficLight=wait_TrafficLight;
GlobVars.SpeedPlanAvoidPedestrian.wait_ped=wait_ped;
GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle=wait_AvoidVehicle;
GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle=wait_avoidOncomingVehicle;
end
