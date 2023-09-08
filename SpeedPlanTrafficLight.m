function [a_soll,GlobVars]=SpeedPlanTrafficLight(speed,d_veh2int,s_b,v_b,greenLight,time2nextSwitch,v_max,GlobVars,Parameters,CalibrationVars)
% greenLight：0 红灯
% greenLight：1 绿灯
% greenLight：2 黄灯
%globalVariable----------------------------------------------------------------------------------------------------------------------
dec_fol=GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight;
dec_bre=GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight;
wait=GlobVars.SpeedPlanTrafficLight.wait_TrafficLight;
%CalibrationVariable----------------------------------------------------------------------------------------------------------------
a_min_com=CalibrationVars.SpeedPlanTrafficLight.a_min_com;%-1.5;
a_max=CalibrationVars.SpeedPlanTrafficLight.a_max;%2.5;
a_min=CalibrationVars.SpeedPlanTrafficLight.a_min;%-3;
v_max_int=CalibrationVars.SpeedPlanTrafficLight.v_max_int;%30/3.6;
% v_max=v_max_int;%实车测试时因信号灯停止线前刹不住，改为场景激活就限速
t_acc=CalibrationVars.SpeedPlanTrafficLight.t_acc;%1.5;
%Parameters--------------------------------------------------------------------------------------------------------------------------
l_veh=Parameters.l_veh;
% w_veh=Parameters.w_veh;
decel=0;
d_veh2int=max([0 d_veh2int]);
d_ist=s_b;
v_soll=v_b;
% 策略模式判断
d_fol=(0-speed.^2)/(2*a_min_com);
d_bre=(0-speed.^2)/(2*a_min);
if dec_fol==0 && dec_bre==0 && d_veh2int>10 && d_veh2int<d_fol
    dec_fol=int16(1);
end
if dec_bre==0
    if d_veh2int<=d_bre+10 && d_veh2int>0 % 原为10 12.31修改1,2023/9/5修改为0 
        dec_bre=int16(1);
    end
else
    if d_veh2int<=1|| wait==1 % 1原为10 12.31修改
        dec_bre=int16(0);
    end
end
if dec_fol==1 && dec_bre==1
    dec_fol=int16(0);
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
            wait=int16(1);
        end
    else
        wait=int16(1);
    end
end
if greenLight~=1 && d_veh2int<=0 && d_veh2int>=-l_veh
    wait=int16(1);
end
% 起步决策
if wait==1 && greenLight==1
    a_predict=min([ACC(v_max,v_soll,d_ist,speed,0,CalibrationVars) a_max]);  % 12.31修改
    s_max=0.5*(min([speed+a_predict*time2nextSwitch v_max_int])+speed)*time2nextSwitch;
    if d_veh2int<30 && s_max>d_veh2int
        wait=int16(0);
        % 12.31修改
        % decel=0;
        % dec_fol=0;
        % dec_bre=0;
    end
end

% ACC速度规划
if dec_fol==0 && dec_bre==0 && d_veh2int>10 && wait==0
    a_soll=ACC(v_max,v_soll,d_ist,speed,wait,CalibrationVars);
else
    if decel==1
%         a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars) ACCcust(v_max_int,0,d_veh2int+CalibrationVars.ACC.d_wait-0.5,speed,a_max,a_min_com,t_acc,CalibrationVars)]);           
        a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars) max([ACC(v_max_int,0,d_veh2int+CalibrationVars.ACC.d_wait,speed,1,CalibrationVars) a_min_com])]);%20220712,去掉ACCcust
    elseif wait==1
        a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars) ACC(v_max_int,0,d_veh2int+CalibrationVars.ACC.d_wait,speed,wait,CalibrationVars)]);
        % a_soll
        % greenLight
    else
        a_soll=ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars);
    end
end
GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight=dec_fol;
GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight=dec_bre;
GlobVars.SpeedPlanTrafficLight.wait_TrafficLight=wait;
end
