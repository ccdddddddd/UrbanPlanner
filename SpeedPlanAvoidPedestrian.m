function [a_soll,d_veh2stopline,GlobVars]=SpeedPlanAvoidPedestrian(pos_s,speed,d_veh2cross,w_cross,s_ped,l_ped,v_ped,psi_ped,s_b,v_b,v_max,GlobVars,Parameters,CalibrationVars)
% pos_s为网联车车头s坐标
% s_ped,l_ped,v_ped,psi_ped皆为大小40的数组，默认值为0,0,-1,0，s_ped按从小到大排列，psi_ped范围为0到360（沿l轴为0度，沿s轴为90度）
% d_veh2cross=max([0 d_veh2cross]);
d_veh2ped=s_ped-pos_s;
for i=1:1:40
    if v_ped(i)<0
        d_veh2ped(i)=200;
    end
end

%globalVariable---------------------------------------------------------------------------------------------------------------------------------------
dec_ped=GlobVars.SpeedPlanAvoidPedestrian.dec_ped;
wait_ped=GlobVars.SpeedPlanAvoidPedestrian.wait_ped;
% CalibrationVariable---------------------------------------------------------------------------------------------------------------------------------
a_max=CalibrationVars.SpeedPlanAvoidPedestrian.a_max;%2.5;
a_min=CalibrationVars.SpeedPlanAvoidPedestrian.a_min;%-2;
v_max_int=CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int;%v_max_in30/3.6;
% v_max_int_emg=CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg;%v_max_int_emg=20/3.6
%Parameters-------------------------------------------------------------------------------------------------------------------------------------------
l_veh=Parameters.l_veh;
w_veh=Parameters.w_veh;
%-----------------------------------------------------------------------------------------------------------------------------------------------------
% v_ped=2;
% v_ped=max([0.00001 2*v_ped]);
% s_ped=abs(s_ped);

% for i=1:1:50
%     v_ped(i)=max([0.00001 v_ped(i)]);
% end

% 策略模式判断
d_bre=(0-speed.^2)/(2*a_min);
if dec_ped==0
    % if min(d_veh2ped(d_veh2ped>=0))<=d_bre+2*l_veh || (d_veh2cross<=d_bre+2*l_veh && d_veh2cross>0) % 1原为l_veh 01.21修改
    if min(d_veh2ped(d_veh2ped>=0))<=d_bre || (d_veh2cross<=d_bre && d_veh2cross>0) % 1原为l_veh 01.21修改
        dec_ped=int16(1);
    end
else
    % if min(d_veh2ped(d_veh2ped>=0))>max(d_bre,(0-v_max_int.^2)/(2*a_min))+3*l_veh && (d_veh2cross==0 || d_veh2cross>max(d_bre,(0-v_max_int.^2)/(2*a_min))+3*l_veh) % 1原为l_veh 01.21修改
    if min(d_veh2ped(d_veh2ped>=0))>max(d_bre,(0-v_max_int.^2)/(2*a_min))+10 && (d_veh2cross==0 || d_veh2cross>max(d_bre,(0-v_max_int.^2)/(2*a_min))+10) % 1原为l_veh 01.21修改
        dec_ped=int16(0);
    end
end
% 停车决策
d_veh2stopline=200;
if dec_ped==1
    for i=1:1:40
        if v_ped(i)>=0 && d_veh2ped(i)>=0 % v_ped(i)默认值为-1
            if psi_ped(i)<=90
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd(psi_ped(i)/2));
            elseif psi_ped(i)>90 && psi_ped(i)<=270
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd((psi_ped(i)+180)/2));
            else
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd((psi_ped(i)+360)/2));
            end
            d_ped=max([d_veh2ped(i)/max([speed 0.00001])*v_abs+0.5*w_veh*1.2 0]);                
            if abs(l_ped(i))<=d_ped
                wait_ped=int16(1);
                d_veh2stopline=d_veh2ped(i);
                break;
            end
        end
    end
end
% 起步决策
if wait_ped==1
    wait_ped=0;
    for i=1:1:40
        if v_ped(i)>=0 && d_veh2ped(i)>=0 % v_ped(i)默认值为-1
            if psi_ped(i)<=90
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd(psi_ped(i)/2));
            elseif psi_ped(i)>90 && psi_ped(i)<=270
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd((psi_ped(i)+180)/2));
            else
                v_abs=max(0,-v_ped(i)*sign(l_ped(i))*cosd((psi_ped(i)+360)/2));
            end
            timeGap=max([0 (abs(l_ped(i))-0.5*w_veh*1.2)/(eps+v_abs)]);
            s_max=0.5*(min([speed+a_max*timeGap v_max_int])+speed)*timeGap;
            % if ~(d_veh2ped(i)<10 && s_max>d_veh2ped(i)) % 是否考虑车长？
            if ~(s_max>d_veh2ped(i)) % 是否考虑车长？
                wait_ped=int16(1);
                break;
            end
        end
    end
end
d_veh2stopline=d_veh2stopline-0.5;%停车位置距行人的距离
if d_veh2stopline>=d_veh2cross && d_veh2stopline<=w_cross+d_veh2cross
    d_veh2stopline=d_veh2cross;
end
% ACC速度规划
if wait_ped==0
    if dec_ped==0 && d_veh2cross>2*l_veh
        a_soll=ACC(v_max,v_b,s_b,speed,wait_ped,CalibrationVars);
    else
        a_soll=ACC(v_max_int,v_b,s_b,speed,wait_ped,CalibrationVars);
    end
else
    a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_ped,CalibrationVars) ACC(v_max_int,0,d_veh2stopline+CalibrationVars.ACC.d_wait-0.5,speed,wait_ped,CalibrationVars)]);
end
    
GlobVars.SpeedPlanAvoidPedestrian.dec_ped=dec_ped;
GlobVars.SpeedPlanAvoidPedestrian.wait_ped=wait_ped;
end

% d_veh2cross<=l_veh || wait_ped==1 “<=l_veh”是否合理
% 过人行横道 有人→停车
