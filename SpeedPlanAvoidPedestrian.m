function [a_soll,dec_ped,wait_ped]=SpeedPlanAvoidPedestrian(speed,dec_ped,d_veh2cross,w_cross,wait_ped,s_ped,v_ped,s_b,v_b,v_max)
% function [a_soll,dec_ped,wait_ped]=SpeedPlanAvoidPedestrian(speed,dec_ped,d_veh2cross,w_cross,wait_ped,s_ped,s_b,v_b,v_max)

d_veh2cross=max([0 d_veh2cross]);

% 设置参数
a_max=2.5;
a_min=-3;
% v_max=14;
v_max_int=30/3.6;
v_max_int_emg=20/3.6;
l_veh=5;
w_veh=1.8;
% v_ped=2;
v_ped=max([0.00001 2*v_ped]);
s_ped=abs(s_ped);
% 策略模式判断
d_bre=(0-speed.^2)/(2*a_min);
if dec_ped==0
    if d_veh2cross<=d_bre+2*l_veh && d_veh2cross>1 % 1原为l_veh 01.21修改
        dec_ped=1;
    end
else
    if d_veh2cross<=1 || wait_ped==1 % 1原为l_veh 01.21修改
        dec_ped=0;
    end
end
% 停车决策
d_ped=max([d_veh2cross/max([speed 0.00001])*v_ped+0.5*w_veh 0]);
if dec_ped==1
    if s_ped<=d_ped
        wait_ped=1;
    end
end
% 起步决策
if wait_ped==1
    timeGap=max([0 (s_ped-0.5*w_veh)/v_ped]);
    s_max=0.5*(min([speed+a_max*timeGap v_max_int])+speed)*timeGap;
    if d_veh2cross<10 && s_max>(w_cross+d_veh2cross)
        wait_ped=0;
    end
end

% ACC速度规划
if wait_ped==0
    a_soll=ACC(v_max_int,v_b,s_b,speed,wait_ped);
elseif dec_ped==1
    a_soll=min([ACC(v_max_int_emg,v_b,s_b,speed,wait_ped) ACC(v_max_int_emg,0,d_veh2cross+3.5+l_veh,speed,wait_ped)]);
else
    a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_ped) ACC(v_max_int,0,d_veh2cross+3.5+l_veh,speed,wait_ped)]);
end
if wait_ped==0 && dec_ped==0 && d_veh2cross>2*l_veh
    a_soll=ACC(v_max,v_b,s_b,speed,wait_ped);
end

end
% d_veh2cross<=l_veh || wait_ped==1 “<=l_veh”是否合理
% 过人行横道 有人→停车
