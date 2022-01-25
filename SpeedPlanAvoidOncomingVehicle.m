function [a_soll,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle]=SpeedPlanAvoidOncomingVehicle(speed,dec_avoidOncomingVehicle,d_veh2waitingArea,wait_avoidOncomingVehicle,s_b,v_b,...,
    s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,v_max)
% 设置参数
a_max=2.5;
a_max_com=1.5;
a_min=-3;
v_max_int=30/3.6;
% v_max_overall=50/3.6;
l_veh=5;
w_veh=1.8;

% 策略模式判断
d_bre=(0-speed.^2)/(2*a_min);
if dec_avoidOncomingVehicle==0
    if d_veh2waitingArea<=d_bre+2*l_veh && d_veh2waitingArea>l_veh
        dec_avoidOncomingVehicle=1;
    end
else
    if d_veh2waitingArea<=0.5*l_veh || wait_avoidOncomingVehicle==1
        dec_avoidOncomingVehicle=0;
    end
end
% 停车决策
d_veh1=max([(d_veh2cross1+l_veh)/max([speed 0.00001])*v_veh1+0.5*w_veh+l_veh 0]);
d_veh2=max([(d_veh2cross2+l_veh)/max([speed 0.00001])*v_veh2+0.5*w_veh+l_veh 0]);
d_veh3=max([(d_veh2cross3+l_veh)/max([speed 0.00001])*v_veh3+0.5*w_veh+l_veh 0]);

if dec_avoidOncomingVehicle==1
    if s_veh1<=d_veh1 || s_veh1apostrophe1>-l_veh || s_veh2<=d_veh2 || s_veh1apostrophe2>-l_veh || s_veh3<=d_veh3 || s_veh1apostrophe3>-l_veh 
        wait_avoidOncomingVehicle=1;
    end
end
% 起步决策
if wait_avoidOncomingVehicle==1
    %     timeGap1=max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.5*(v_veh1+v_max_overall) 0.00001])]);
    %     if (v_max_overall-v_veh1)>a_max_com*timeGap1
    %         fun_x=@(x)v_veh1*x+0.5*a_max_com*(x.^2);
    %         [timeGap1,~,~] = fzero(@(x)fun_x(x)-(s_veh1-0.5*w_veh-l_veh),[0 max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.00001])])]);
    %     end
    timeGap1=max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.00001])]);
    s_max1=0.5*(min([speed+a_max_com*timeGap1 v_max_int])+speed)*timeGap1;
    timeGap2=max([0 (s_veh2-0.5*w_veh-l_veh)/max([v_veh2 0.00001])]);
    s_max2=0.5*(min([speed+a_max_com*timeGap2 v_max_int])+speed)*timeGap2;
    timeGap3=max([0 (s_veh3-0.5*w_veh-l_veh)/max([v_veh3 0.00001])]);
    s_max3=0.5*(min([speed+a_max_com*timeGap3 v_max_int])+speed)*timeGap3;
    if d_veh2waitingArea<10 && s_max1>d_veh2cross1+l_veh && s_veh1apostrophe1<-l_veh && s_max2>d_veh2cross2+l_veh && s_veh1apostrophe2<-l_veh && s_max3>d_veh2cross3+l_veh && s_veh1apostrophe3<-l_veh
        wait_avoidOncomingVehicle=0;
    end
end

% ACC速度规划
if wait_avoidOncomingVehicle==1
    a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,d_veh2waitingArea+4+l_veh,speed,wait_avoidOncomingVehicle)]);
else
    if dec_avoidOncomingVehicle==1
    a_soll=ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle);
    else
        if d_veh2waitingArea>2*l_veh
            a_soll=ACC(v_max,v_b,s_b,speed,wait_avoidOncomingVehicle);
        else
            a_soll=ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle);
        end
    end
end
% v_veh1
% a_soll
% if d_veh2waitingArea<5.4
%     s_veh1
% end
% d_veh2waitingArea
end
% 函数调用
% speed=10;
% dec_avoidOncomingVehicle=0;
% d_veh2waitingArea=20;
% wait_avoidOncomingVehicle=0;
% s_b=100;
% v_b=10;
% s_veh1=40;
% v_veh1=5;
% d_veh2cross1=30;
% s_veh1apostrophe1=-10;
% s_veh2=200;
% v_veh2=0;
% d_veh2cross2=0;
% s_veh1apostrophe2=-200;
% s_veh3=200;
% v_veh3=0;
% d_veh2cross3=0;
% s_veh1apostrophe3=-200;
% v_max=50/3.6;
% [a_soll,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle]=SpeedPlanAvoidOncomingVehicle(speed,dec_avoidOncomingVehicle,d_veh2waitingArea,wait_avoidOncomingVehicle,s_b,v_b,...,
%     s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,v_max);