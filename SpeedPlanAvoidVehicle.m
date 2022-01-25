function [a_soll,dec_fol,dec_bre,wait]=SpeedPlanAvoidVehicle(speed,dec_fol,dec_bre,d_veh2int,d_veh2stopline,wait,s_a,v_a,s_b,v_b,s_c,v_c)
v_a=max([0.00001 v_a]);
v_b=max([0.00001 v_b]);
v_c=max([0.00001 v_c]);
% 设置参数
a_min_com=-1.5;
a_max=1.5;
a_min=-3;
v_max=40/3.6;
l_veh=5;
w_veh=1.8;
t_re=1.5;
GapIndex=2;
% t_acc=1.5;

d_ist=s_a;
v_soll=v_a;
% 策略模式判断
d_fol=(0-speed.^2)/(2*a_min_com);
d_bre=(0-speed.^2)/(2*a_min);
if dec_fol==0 && dec_bre==0 && d_veh2stopline>0 && d_veh2stopline<=d_fol
    dec_fol=1;
end
if dec_bre==0
    if d_veh2stopline<=min([d_bre+15 20]) && d_veh2stopline>0
        dec_bre=1;
    end
else
    if d_veh2stopline<=0 || wait==1
        dec_bre=0;
    end
end
if dec_fol==1 && dec_bre==1
    dec_fol=0;
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
        prereq1=(s_a_end-s_int-l_veh-l_veh>max([0 v_b*t_re (v_a.^2-v_b.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_b*t_re/GapIndex) < min(s_max, s_a_end-l_veh-v_b*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=a
            d_ist=s_a;
            v_soll=v_a;
        else
            s_b_end=s_b+t_c2int*v_b;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_b2int<t_c2int; % b在c先
            prereq2=(s_b_end-s_int-l_veh-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_veh-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3
                % 前车=b
                d_ist=s_b;
                v_soll=v_b;
            else
                % 前车=c
                d_ist=s_c;
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
        prereq1=(s_b_end-s_int-l_veh-l_veh>max([0 v_a*t_re (v_b.^2-v_a.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_a*t_re/GapIndex) < min(s_max, s_b_end-l_veh-v_a*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=b
            d_ist=s_b;
            v_soll=v_b;
        else
            s_a_end=s_a+t_c2int*v_a;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_a2int<t_c2int; % a在c先
            prereq2=(s_a_end-s_int-l_veh-l_veh>max([0 v_c*t_re (v_a.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_a_end-l_veh-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3
                % 前车=a
                d_ist=s_a;
                v_soll=v_a;
            else
                % 前车=c
                d_ist=s_c;
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
        prereq1=(s_a_end-s_int-l_veh-l_veh>max([0 v_b*t_re (v_a.^2-v_b.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_b*t_re/GapIndex) < min(s_max, s_a_end-l_veh-v_b*t_re/GapIndex));
        if prereq1&&prereq2
%             prereq1
%             prereq2
            % 前车=a
            d_ist=s_a;
            v_soll=v_a;
        else
            s_b_end=s_b+t_c2int*v_b;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_b2int<t_c2int; % b在c先
            prereq2=s_b_end-s_int-l_veh-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]);
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_veh-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3
                % 前车=b
                d_ist=s_b;
                v_soll=v_b;
            else
                % 无解
                wait=1;
            end
        end
    else
        % b在a先
        s_b_end=s_b+t_a2int*v_b;
        % s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2);
        % s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2);
        s_max=0.5*(min([speed+a_max*t_a2int v_max])+speed)*t_a2int;
        s_min=0.5*(max([speed+a_min*t_a2int 0])+speed)*t_a2int;
        prereq1=(s_b_end-s_int-l_veh-l_veh>max([0 v_a*t_re (v_b.^2-v_a.^2)/(2*a_min)]));
        prereq2=(max(s_min, s_int+l_veh+v_a*t_re/GapIndex) < min(s_max, s_b_end-l_veh-v_a*t_re/GapIndex));
        if prereq1&&prereq2
            % 前车=b
            d_ist=s_b;
            v_soll=v_b;
        else
            s_a_end=s_a+t_c2int*v_a;
            % s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2);
            % s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2);
            s_max=0.5*(min([speed+a_max*t_c2int v_max])+speed)*t_c2int;
            s_min=0.5*(max([speed+a_min*t_c2int 0])+speed)*t_c2int;
            prereq1=t_a2int<t_c2int; % a在c先
            prereq2=(s_a_end-s_int-l_veh-l_veh>max([0 v_c*t_re (v_a.^2-v_c.^2)/(2*a_min)]));
            prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_a_end-l_veh-v_c*t_re/GapIndex));
            if prereq1&&prereq2&&prereq3
                % 前车=a
                d_ist=s_a;
                v_soll=v_a;
            else
                % 无解
                wait=1;
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
    prereq2=s_b_end-s_int-l_veh-l_veh>max([0 v_c*t_re (v_b.^2-v_c.^2)/(2*a_min)]);
    prereq3=(max(s_min, s_int+l_veh+v_c*t_re/GapIndex) < min(s_max, s_b_end-l_veh-v_c*t_re/GapIndex));
    prereq4=(s_a>10)&&(d_veh2stopline<15);
    % prereq5=s_c<-l_veh/2;
    % if prereq1&&prereq2&&prereq3&&prereq4&&prereq5
    if prereq1&&prereq2&&prereq3&&prereq4
        % 前车=b
        d_ist=s_b;
        v_soll=v_b;
        wait=0;
    end
end

% ACC速度规划
% a_acc=min([ACC(v_max,v_soll,d_ist,speed) ACC(v_max,v_a,s_a,speed)]);
a_acc=min([ACC(v_max,v_soll,d_ist,speed,wait) ACC(v_max,v_a,s_a,speed,wait)]);
if wait==0
    a_soll=a_acc;
else
    % 停车待通行状态下速度规划
    a_dec=ACC(v_max,0,max([0 d_veh2stopline]),speed,wait);
    a_soll=min([a_dec ACC(v_max,v_a,s_a,speed,wait)]);
end
if dec_fol==1
    a_soll=max([a_soll -2]);
end
end
