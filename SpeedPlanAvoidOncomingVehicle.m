function [a_soll,GlobVars]=SpeedPlanAvoidOncomingVehicle(speed,d_veh2waitingArea,s_b,v_b,...,
    s_veh,v_veh,d_veh2conflict,s_vehapostrophe,v_max,GlobVars,CalibrationVars,Parameters)
%globalVariable----------------------------------------------------------------------------------------------------------------------
dec_avoidOncomingVehicle=GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle;
wait_avoidOncomingVehicle=GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;
% 设置参数
% a_max=2.5;
a_max_com=CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com;%1.5;
a_min=CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min;%-3;
v_max_int=CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int;%30/3.6;
D_safe=CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe;%2
% v_max_overall=50/3.6;
l_veh=Parameters.l_veh;
w_veh=Parameters.w_veh;

d_veh=zeros(1,6);
timeGap=zeros(1,6);
s_max=zeros(1,6);
% 策略模式判断
d_bre=(0-speed.^2)/(2*a_min);
if dec_avoidOncomingVehicle==0
    if d_veh2waitingArea<=d_bre+2*l_veh && d_veh2waitingArea>l_veh
        dec_avoidOncomingVehicle=int16(1);
    end
else
    if d_veh2waitingArea<=0.5*l_veh || wait_avoidOncomingVehicle==1
        dec_avoidOncomingVehicle=int16(0);
    end
end
% 停车决策
for i=1:1:6
    d_veh(i)=max([(d_veh2conflict(i)+l_veh)/max([speed 0.00001])*v_veh(i)+0.5*w_veh+D_safe 0]);
end
if dec_avoidOncomingVehicle==1
    for i=1:1:6
%         if s_veh(i)<=d_veh(i) || s_vehapostrophe(i)>-l_veh
        if s_veh(i)<=d_veh(i) || s_vehapostrophe(i)>0%20220704
            wait_avoidOncomingVehicle=int16(1);
            break;
        end
    end
end
% 起步决策
if wait_avoidOncomingVehicle==1
    %     timeGap1=max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.5*(v_veh1+v_max_overall) 0.00001])]);
    %     if (v_max_overall-v_veh1)>a_max_com*timeGap1
    %         fun_x=@(x)v_veh1*x+0.5*a_max_com*(x.^2);
    %         [timeGap1,~,~] = fzero(@(x)fun_x(x)-(s_veh1-0.5*w_veh-l_veh),[0 max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.00001])])]);
    %     end
    for i=1:1:6
        timeGap(i)=max([0 (s_veh(i)-0.5*w_veh-D_safe)/max([v_veh(i) 0.00001])]);
        s_max(i)=0.5*(min([speed+a_max_com*timeGap(i) v_max_int])+speed)*timeGap(i);
    end
    wait_avoidOncomingVehicle=int16(0);
    for i=1:1:6
%         if ~(d_veh2waitingArea<10 && s_max(i)>d_veh2conflict(i)+l_veh && s_vehapostrophe(i)<-l_veh)
        if ~(d_veh2waitingArea<10 && s_max(i)>d_veh2conflict(i)+l_veh && s_vehapostrophe(i)<0)%20220704
            wait_avoidOncomingVehicle=int16(1);
            break;
        end
    end
end

% ACC速度规划
if wait_avoidOncomingVehicle==1
    % a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]);
    a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle,CalibrationVars) ACC(v_max_int,0,max([0 d_veh2waitingArea+CalibrationVars.ACC.d_wait]),speed,wait_avoidOncomingVehicle,CalibrationVars)]);
else
    if dec_avoidOncomingVehicle==1
    a_soll=ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle,CalibrationVars);
    else
        if d_veh2waitingArea>2*l_veh
            a_soll=ACC(v_max,v_b,s_b,speed,wait_avoidOncomingVehicle,CalibrationVars);
        else
            a_soll=ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle,CalibrationVars);
        end
    end
end
GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle=dec_avoidOncomingVehicle;
GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle=wait_avoidOncomingVehicle;
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