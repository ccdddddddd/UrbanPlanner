function [a_soll,GlobVars]=SpeedPlanAvoidVehicle(speed,d_veh2int,d_veh2stopline,s_a,v_a,l_a,s_b,v_b,l_b,s_c,v_c,l_c,v_max,GlobVars,CalibrationVars,Parameters)
% CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle
%globalVariable----------------------------------------------------------------------------------------------------------------------
dec_fol=GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle;
dec_bre=GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle;
wait=GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle;
v_a=max([0.00001 v_a]);
v_b=max([0.00001 v_b]);
v_c=max([0.00001 v_c]);
%CalibrationVariable----------------------------------------------------------------------------------------------------------------
a_min_com=CalibrationVars.SpeedPlanAvoidVehicle.a_min_com;%-1.5;
a_max=CalibrationVars.SpeedPlanAvoidVehicle.a_max;%1.5;
a_min=CalibrationVars.SpeedPlanAvoidVehicle.a_min;%-3;
t_re=CalibrationVars.SpeedPlanAvoidVehicle.t_re;%1.5;
GapIndex=CalibrationVars.SpeedPlanAvoidVehicle.gapIndex;%2;
minDis4DecBre = CalibrationVars.SpeedPlanAvoidVehicle.minDis4DecBre;% = 5;
if (s_b<999 || s_c>-999) && (v_b<0.75*CalibrationVars.SpeedPlanAvoidVehicle.v_max || v_c<0.75*CalibrationVars.SpeedPlanAvoidVehicle.v_max) %主路有车,且车速小于30km/h
    v_max = min(v_max,0.75*CalibrationVars.SpeedPlanAvoidVehicle.v_max);%40/3.6;
else
    v_max=min(v_max,CalibrationVars.SpeedPlanAvoidVehicle.v_max);%40/3.6;
end
%Parameters--------------------------------------------------------------------------------------------------------------------------
l_veh=Parameters.l_veh;
% w_veh=1.8;

% t_acc=1.5;

d_ist=s_a-l_a;
v_soll=v_a;
% 策略模式判断
d_fol=(0-speed.^2)/(2*a_min_com);
d_bre=(0-speed.^2)/(2*a_min);
if dec_fol==0 && dec_bre==0 && d_veh2stopline>0 && d_veh2stopline<=d_fol
    dec_fol=int16(1);
end
if dec_bre==0
    % if d_veh2stopline<=min([d_bre+15 20]) && d_veh2stopline>0
    if d_veh2stopline<=min([d_bre+10 15]) && d_veh2stopline>minDis4DecBre %minDis4DecBre
        dec_bre=int16(1);
    end
else
    if d_veh2stopline<=minDis4DecBre || wait==1
        dec_bre=int16(0);
    end
end
if dec_fol==1 && dec_bre==1 || d_veh2stopline<0
    dec_fol=int16(0);
end

s_int=d_veh2int;
t_a2int=(s_int-s_a)/v_a;
t_b2int=(s_int-s_b)/v_b;
t_c2int=(s_int-s_c)/v_c;
% 跟车决策
if dec_fol==1
    if t_a2int<t_b2int
        % a在b先
        s_a_end=s_a+t_b2int*v_a;
        % s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2);
        s_max=0.5*(min([speed+a_max*t_b2int v_max])+speed)*t_b2int;
        % s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2);
        s_min=0.5*(max([speed+a_min*t_b2int 0])+speed)*t_b2int;
        prereq1=(s_a_end-s_int-l_a-l_veh>max([0 v_b*t_re (v_a.^2-v_b.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_b*t_re/GapIndex) < min(s_max, s_a_end-l_a-v_b*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=a
            d_ist=s_a-l_a;
            v_soll=v_a;
        else
            s_b_end=s_b+t_c2int*v_b;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_b2int<t_c2int; % b在c先
            prereq2=(s_b_end-s_int-l_b-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_b-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3 || (s_c<=-200)
                % 前车=b
                d_ist=s_b-l_b;
                v_soll=v_b;
            else
                % 前车=c
                d_ist=s_c-l_c;
                v_soll=v_c;
            end
        end
    else
        % b在a先
        s_b_end=s_b+t_a2int*v_b;
        % s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2);
        % s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2);
        s_max=0.5*(min([speed+a_max*t_a2int v_max])+speed)*t_a2int;
        s_min=0.5*(max([speed+a_min*t_a2int 0])+speed)*t_a2int;
        prereq1=(s_b_end-s_int-l_b-l_veh>max([0 v_a*t_re (v_b.^2-v_a.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_a*t_re/GapIndex) < min(s_max, s_b_end-l_b-v_a*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=b
            d_ist=s_b-l_b;
            v_soll=v_b;
        else
            s_a_end=s_a+t_c2int*v_a;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_a2int<t_c2int; % a在c先
            prereq2=(s_a_end-s_int-l_a-l_veh>max([0 v_c*t_re (v_a.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_a_end-l_a-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3 || (s_c<=-200)
                % 前车=a
                d_ist=s_a-l_a;
                v_soll=v_a;
            else
                % 前车=c
                d_ist=s_c-l_c;
                v_soll=v_c;
            end
        end
    end
%     s_a,v_a,s_b,v_b,s_c,v_c
%                     d_ist
%                     v_soll
%                 v_soll

end
% 停车决策
if dec_bre==1
    s_int=d_veh2int;
    t_a2int=(s_int-s_a)/v_a;

    t_b2int=(s_int-s_b)/v_b;
    t_c2int=(s_int-s_c)/v_c;
    if t_a2int<t_b2int
        % a在b先
        s_a_end=s_a+t_b2int*v_a;
        % s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2);
        % s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2);
        s_max=0.5*(min([speed+a_max*t_b2int v_max])+speed)*t_b2int;
        s_min=0.5*(max([speed+a_min*t_b2int 0])+speed)*t_b2int;
        prereq1=(s_a_end-s_int-l_a-l_veh>max([0 v_b*t_re (v_a.^2-v_b.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_b*t_re/GapIndex) < min(s_max, s_a_end-l_a-v_b*t_re/GapIndex));
        if prereq1&&prereq2
%             prereq1
%             prereq2
            % 前车=a
            d_ist=s_a-l_a;
            v_soll=v_a;
        else
            s_b_end=s_b+t_c2int*v_b;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_b2int<t_c2int; % b在c先
            prereq2=s_b_end-s_int-l_b-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]);
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_b-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3 || (s_c<=-200)
                % 前车=b
                d_ist=s_b-l_b;
                v_soll=v_b;
            else
                % 无解
                wait=int16(1);
            end
        end
    else
        % b在a先
        s_b_end=s_b+t_a2int*v_b;
        % s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2);
        % s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2);
        s_max=0.5*(min([speed+a_max*t_a2int v_max])+speed)*t_a2int;
        s_min=0.5*(max([speed+a_min*t_a2int 0])+speed)*t_a2int;
        prereq1=(s_b_end-s_int-l_b-l_veh>max([0 v_a*t_re (v_b.^2-v_a.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_a*t_re/GapIndex) < min(s_max, s_b_end-l_b-v_a*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=b
            d_ist=s_b-l_b;
            v_soll=v_b;
        else
            s_a_end=s_a+t_c2int*v_a;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_a2int<t_c2int; % a在c先
            prereq2=(s_a_end-s_int-l_a-l_veh>max([0 v_c*t_re (v_a.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_a_end-l_a-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3  || (s_c<=-200)
                % 前车=a
                d_ist=s_a-l_a;
                v_soll=v_a;
            else
                % 无解
                wait=int16(1);
            end
        end
    end
end
% 起步决策
if wait==1
    s_b_end=s_b+t_c2int*v_b;
    % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
    % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
    s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
    s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
    prereq1=t_b2int<t_c2int; % b在c先
    prereq2=s_b_end-s_int-l_b-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]);
    prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_b-v_c*t_re/GapIndex));
    prereq4=(s_a>10)&&(d_veh2stopline<10);
    prereq5=ACC(v_max,v_b,s_b-l_b,speed,0,CalibrationVars) > 0.25;
    % prereq4=(s_a>10)&&(d_veh2stopline<15);
    if prereq1&&prereq2&&prereq3&&prereq4&&prereq5
        % 前车=b
        d_ist=s_b-l_b;
        v_soll=v_b;
        wait=int16(0);
    end
end

% ACC速度规划
% a_acc=min([ACC(v_max,v_soll,d_ist,speed) ACC(v_max,v_a,s_a,speed)]);
a_acc=min([ACC(v_max,v_soll,d_ist,speed,wait,CalibrationVars) ACC(v_max,v_a,s_a-l_a,speed,wait,CalibrationVars)]);
if wait==0
    if d_veh2stopline<=0
        a_soll=min([a_acc ACC(v_max,v_b,s_b-l_b,speed,wait,CalibrationVars)]);
    else
        a_soll=a_acc;
    end
else
    % 停车待通行状态下速度规划
    a_dec=ACC(v_max,0,max([0 d_veh2stopline+CalibrationVars.ACC.d_wait]),speed,wait,CalibrationVars);
    a_soll=min([a_dec ACC(v_max,v_a,s_a-l_a,speed,wait,CalibrationVars)]);
end
if dec_fol==1%跟车状态下，跟主路车加速度和停车加速度取最大且与匝道前车加速度取最小
    a_soll=min(ACC(v_max,v_a,s_a-l_a,speed,wait,CalibrationVars),max([a_soll ACC(v_max,0,max([0 d_veh2stopline+CalibrationVars.ACC.d_wait]),speed,1,CalibrationVars)]));
end
GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle=dec_fol;
GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle=dec_bre;
GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle=wait;
end
