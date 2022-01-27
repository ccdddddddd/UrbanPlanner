function [AEBActive,wait_ped,wait_AvoidVehicle,wait_avoidOncomingVehicle]=AEBDecision(AEBActive,PrePedestrianActive,PedestrianActive,wait_ped,wait_AvoidVehicle,wait_avoidOncomingVehicle,speed,d_veh2stopline,...,
    d_veh2waitingArea,s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3)
% 紧急停车决策
l_veh=5;
w_veh=1.8;
if AEBActive==0
    if PrePedestrianActive==1 && PedestrianActive==0 && wait_ped==1 && speed>0 % 避让行人决策 → AEB
        AEBActive=1; 
        wait_ped=0;
    end
    if d_veh2stopline<=0 && wait_AvoidVehicle==1 && speed>0 && AEBActive==0 % 避让同向车辆决策 → AEB
        AEBActive=2; 
        wait_AvoidVehicle=0;
    end
    if d_veh2waitingArea<=0 && speed>0 && AEBActive==0 % 避让对向车辆决策 → AEB
        if wait_avoidOncomingVehicle==1
            AEBActive=3; 
            wait_avoidOncomingVehicle=0;
        else
            if d_veh2cross1==0
                s_veh1=200;
                v_veh1=0;
                s_veh1apostrophe1=-200;
            end
            if d_veh2cross2==0
                s_veh2=200;
                v_veh2=0;
                s_veh1apostrophe2=-200;
            end
            if d_veh2cross3==0
                s_veh3=200;
                v_veh3=0;
                s_veh1apostrophe3=-200;
            end
            d_veh1=max([(d_veh2cross1+l_veh)/max([speed 0.00001])*v_veh1+0.5*w_veh+l_veh 0]);
            d_veh2=max([(d_veh2cross2+l_veh)/max([speed 0.00001])*v_veh2+0.5*w_veh+l_veh 0]);
            d_veh3=max([(d_veh2cross3+l_veh)/max([speed 0.00001])*v_veh3+0.5*w_veh+l_veh 0]);
            if s_veh1<=d_veh1 || s_veh1apostrophe1>-l_veh || s_veh2<=d_veh2 || s_veh1apostrophe2>-l_veh || s_veh3<=d_veh3 || s_veh1apostrophe3>-l_veh
                AEBActive=3;
            end
        end        
    end
end

end
