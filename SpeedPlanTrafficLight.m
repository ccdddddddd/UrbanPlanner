function [a_soll,dec_fol,dec_bre,wait]=SpeedPlanTrafficLight(speed,dec_fol,dec_bre,d_veh2int,wait,s_b,v_b,greenLight,time2nextSwitch)
% 设置参数
a_min_com=-1.5;
a_max=2.5;
a_min=-3;
v_max=14;
% v_max_int=36/3.6;
v_max_int=30/3.6;
l_veh=5;
w_veh=1.8;
t_acc=1.5;
decel=0;

d_ist=s_b;
v_soll=v_b;
% 策略模式判断
d_fol=(0-speed.^2)/(2*a_min_com);
d_bre=(0-speed.^2)/(2*a_min);
if dec_fol==0 && dec_bre==0 && d_veh2int>10 && d_veh2int<d_fol
    dec_fol=1;
end
if dec_bre==0
    if d_veh2int<=d_bre+10 && d_veh2int>1 % 1原为10 12.31修改
        dec_bre=1;
    end
else
    if d_veh2int<=1|| wait==1 % 1原为10 12.31修改
        dec_bre=0;
    end
end
if dec_fol==1 && dec_bre==1
    dec_fol=0;
end


% 减速决策
if dec_fol==1
    if greenLight==1
        if d_veh2int/speed<time2nextSwitch
            decel=0;
        else
            decel=1;
        end
    else
        decel=1;
    end
end
% 停车决策
if dec_bre==1
    if greenLight==1
%         if d_veh2int/min([0.5*(v_max_int+speed) speed])>=time2nextSwitch
        if d_veh2int/speed>=time2nextSwitch
            wait=1;
        end
    else
        wait=1;
    end
end
% 起步决策
if wait==1 && greenLight==1
    a_predict=min([ACC(v_max,v_soll,d_ist,speed,0) a_max]);  % 12.31修改
    s_max=0.5*(min([speed+a_predict*time2nextSwitch v_max_int])+speed)*time2nextSwitch;
    if d_veh2int<30 && s_max>d_veh2int
        wait=0;
        % 12.31修改
        % decel=0;
        % dec_fol=0;
        % dec_bre=0;
    end
end

% ACC速度规划
if dec_fol==0 && dec_bre==0 && d_veh2int>10 && wait==0
    a_soll=ACC(v_max,v_soll,d_ist,speed,wait);
else
    if decel==1
        a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait) ACCcust(v_max_int,0,d_veh2int+3.5+l_veh,speed,a_max,a_min_com,t_acc)]);
    elseif wait==1
        a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait) ACC(v_max_int,0,d_veh2int+3.5+l_veh,speed,wait)]);
        % a_soll
        % greenLight
    else
        a_soll=ACC(v_max_int,v_soll,d_ist,speed,wait);
    end
end
end
