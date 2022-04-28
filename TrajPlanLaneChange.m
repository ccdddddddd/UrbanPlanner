function [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
    TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,LeftLaneBehindDis,LeftLaneBehindVel,LeftLaneFrontDis,LeftLaneFrontVel,RightLaneBehindDis,RightLaneBehindVel,RightLaneFrontDis,RightLaneFrontVel,speed,...,
    pos_s,pos_l_CurrentLane,CurrentLaneIndex,TargetLaneIndex,BackupTargetLaneIndex,d_veh2int,WidthOfLanes,v_max,LanesWithFail,GlobVars,CalibrationVars,Parameters)
%globalVariable----------------------------------------------------------------------------------------------------------------------
CountLaneChange=GlobVars.TrajPlanLaneChange.CountLaneChange;
DurationLaneChange=GlobVars.TrajPlanLaneChange.DurationLaneChange;
LaneChangePath=GlobVars.TrajPlanLaneChange.LaneChangePath;
t_lc_traj=GlobVars.TrajPlanLaneChange.t_lc_traj;
CurrentTargetLaneIndex=GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex;
w_lane_left=0.5*WidthOfLanes(max(CurrentLaneIndex-1,1))+0.5*WidthOfLanes(CurrentLaneIndex);
w_lane_right=0.5*WidthOfLanes(min(CurrentLaneIndex+1,6))+0.5*WidthOfLanes(CurrentLaneIndex);
% w_lane=3.2;
if TargetLaneIndex<=CurrentLaneIndex
    TargetLaneBehindDis=LeftLaneBehindDis;
    TargetLaneBehindVel=LeftLaneBehindVel;
    TargetLaneFrontDis=LeftLaneFrontDis;
    TargetLaneFrontVel=LeftLaneFrontVel;
else
    TargetLaneBehindDis=RightLaneBehindDis;
    TargetLaneBehindVel=RightLaneBehindVel;
    TargetLaneFrontDis=RightLaneFrontDis;
    TargetLaneFrontVel=RightLaneFrontVel;
end
if BackupTargetLaneIndex~=-1
    s_d=RightLaneBehindDis;
    v_d=RightLaneBehindVel;
    s_e=RightLaneFrontDis;
    v_e=RightLaneFrontVel;
    BackupTargetLaneIndex=int16(min([CurrentLaneIndex+1 BackupTargetLaneIndex]));
    BackupTargetLaneIndex=int16(max([CurrentLaneIndex-1 BackupTargetLaneIndex]));
else
    s_d=-200;
    v_d=-20;
    s_e=200;
    v_e=20;
end

% TargetLaneIndex=TargetLaneIndex-1;
% CurrentLaneIndex=CurrentLaneIndex-1;
s_a=CurrentLaneFrontDis;
v_a=CurrentLaneFrontVel;
s_b=TargetLaneBehindDis;
v_b=TargetLaneBehindVel;
s_c=TargetLaneFrontDis;
v_c=TargetLaneFrontVel;
v_max_int=CalibrationVars.TrajPlanLaneChange.v_max_int;%30/3.6;
indexAfterLaneChangeDis2Int=CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int;%1;
t_permit=CalibrationVars.TrajPlanLaneChange.t_permit;%3;
t_re=CalibrationVars.TrajPlanLaneChange.t_re;%0.5;
index_accel=CalibrationVars.TrajPlanLaneChange.index_accel;%0.5;
a_max_comfort=CalibrationVars.TrajPlanLaneChange.a_max_comfort;%1;
a_min=CalibrationVars.TrajPlanLaneChange.a_min;%-3.5;
a_max=CalibrationVars.TrajPlanLaneChange.a_max;%2.5;
a_min_comfort=CalibrationVars.TrajPlanLaneChange.a_min_comfort;%-1;
a_soll=100;
% t_lc=max([2-0.05*(speed-10) 1.7]);
t_lc=max([2-0.04*(speed-15) 2]);
% t_lc=min([t_lc 2.3]);
t_lc=min([t_lc 2.5]);
t_lc=ceil(t_lc/0.1)*0.1;
traj=zeros([6 120]);
l_veh=Parameters.l_veh;
% w_veh=1.8;
TargetLaneIndex=min([CurrentLaneIndex+1 TargetLaneIndex]);
TargetLaneIndex=max([CurrentLaneIndex-1 TargetLaneIndex]);
S_traj=zeros(1,4/0.05);
X_traj=zeros(1,4/0.05);
V_traj=zeros(1,4/0.05);

wait=0;
if isempty(LanesWithFail)==0
    for i=LanesWithFail
        if CurrentLaneIndex==i
            wait=-1;
        end
    end
end

% Lane change decision
S_0=0;
V_0=speed;
S_end=0;%2020324,编译c报错增加初始值
V_end=0;
if CountLaneChange==0 && CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex
%     if TargetLaneIndex<=CurrentLaneIndex
%         w_lane=w_lane_left;
%     else
%         w_lane=w_lane_right;
%     end
    w_lane=0.5*WidthOfLanes(CurrentLaneIndex)+0.5*WidthOfLanes(TargetLaneIndex);%20220328

    if speed<5
        S_end=max([10 t_lc*speed]);
        fun_t=@(t)(speed*t+0.5*2*t.^2);
        [t_lc_strich,~,~] = fzero(@(t)fun_t(t)-(S_end.^2+w_lane.^2).^0.5,[1 4]);
        t_lc=max([t_lc t_lc_strich]);
        t_lc=0.1*round(t_lc/0.1);
        S_min_dyn=max([(max([w_lane (0-V_0.^2)/(2*a_min)]).^2-w_lane.^2).^0.5 (max([w_lane S_0+V_0*t_lc+0.5*a_min*t_lc*t_lc]).^2-w_lane.^2).^0.5]);
        
        index_accel_strich=max([ACC(v_max,v_c,s_c-s_b,v_b,0,CalibrationVars)/a_max_comfort 0.5]);
        V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]);
        S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc;
        V_c_end=max([0 v_c+(index_accel*a_min_comfort)*t_lc]);
        S_c_end=s_c+0.5*(V_c_end+v_c)*t_lc;
        V_b_end=max([0 v_b+(index_accel_strich*a_max_comfort)*t_lc]);
        S_b_end=s_b+0.5*(V_b_end+v_b)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        % S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc;
        % S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % V_c_end=v_c+(index_accel*a_min_comfort)*t_lc;
        % V_b_end=v_b+(index_accel_strich*a_max_comfort)*t_lc;

        if (-s_b>v_b*t_lc) && ...,
                (s_c<=V_0*t_lc)
            %                 b车不存在 c车存在
            % V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif ( -s_b<=v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车存在 c车不存在
            % V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif (-s_b>v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车不存在 c车不存在
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        else
            %                 b车存在 c车存在
            % V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+max([V_b_end-V_end 0])*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        end
        prereq1=(S_c_end-S_b_end-l_veh-l_veh>V_b_end*t_re+(V_c_end.^2-V_b_end.^2)/(2*a_min));
        prereq2=(S_a_end>0.5*(S_0+S_end+l_veh+l_veh));
        prereq3=(V_end>V_0+a_min*t_lc);
        prereq4=(V_end<V_0+a_max*t_lc);
        prereq5=(S_max>S_end&&S_end>S_min);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh; % 距离路口过近时不允许换道
        prereq7=(s_b<=-l_veh);
        prereq8=(s_c>=l_veh);
        prereq9=(S_a_mid>0.5*S_end+l_veh);
        prereq10=(s_a>=l_veh+0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq2&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    else
        index_accel_strich=max([ACC(v_max,v_c,s_c-s_b,v_b,0,CalibrationVars)/a_max_comfort 0.5]);
        V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]);
        S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc;
        V_c_end=max([0 v_c+(index_accel*a_min_comfort)*t_lc]);
        S_c_end=s_c+0.5*(V_c_end+v_c)*t_lc;
        V_b_end=max([0 v_b+(index_accel_strich*a_max_comfort)*t_lc]);
        S_b_end=s_b+0.5*(V_b_end+v_b)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;

        % S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc;
        % S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
        % V_c_end=v_c+(index_accel*a_min_comfort)*t_lc;
        % V_b_end=v_b+(index_accel*index_accel_strich)*t_lc;
        if (-s_b>v_b*t_lc) && ...,
                (s_c<=V_0*t_lc)
            %                 b车不存在 c车存在
            V_end=min([0.5*(V_0+V_c_end) V_0]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif ( -s_b<=v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车存在 c车不存在
            V_end=min([0.5*(V_0+V_b_end) V_0]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif (-s_b>v_b*t_lc) && ...,
                ( s_c>V_0*t_lc)
            %                 b车不存在 c车不存在
            S_end=V_0*t_lc;
            V_end=V_0;
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
        else
            %                 b车存在 c车存在
            V_end=min([V_0 0.5*(V_b_end+V_c_end)]);
            S_min=max([S_b_end+V_b_end*t_re+l_veh S_b_end+V_b_end*t_re+(V_end.^2-V_b_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        end
        % 参见TrajPlanLaneChange_S_max_withAccel.bmp
        S_max_withAccel=t_lc*min([V_end V_0])+0.5*a_max_comfort*t_lc*t_lc-((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 
        S_min_withAccel=t_lc*max([V_end V_0])-0.5*a_max_comfort*t_lc*t_lc+((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移
        prereq1=(S_c_end-S_b_end-l_veh-l_veh>V_b_end*t_re+(V_c_end.^2-V_b_end.^2)/(2*a_min));
        prereq2=(S_a_end>0.5*(S_0+S_end+l_veh+l_veh));
        prereq3=(V_end>=V_0+a_min_comfort*t_lc);
        prereq4=(V_end<=V_0+a_max_comfort*t_lc);
        prereq5=(S_max>=S_min&&S_end<=S_max_withAccel&&S_end>=S_min_withAccel);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh; % 距离路口过近时不允许换道
        % prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道
        prereq7=(s_b<=-l_veh);
        prereq8=(s_c>=l_veh);
        prereq9=(S_a_mid>0.5*S_end+l_veh);
        prereq10=(s_a>=l_veh+0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq2&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    end
end
if CountLaneChange==0 && CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && -1~=BackupTargetLaneIndex
    if BackupTargetLaneIndex<=CurrentLaneIndex
        w_lane=w_lane_left;
    else
        w_lane=w_lane_right;
    end
    if speed<5
        S_end=max([10 t_lc*speed]);
        fun_t=@(t)(speed*t+0.5*2*t.^2);
        [t_lc_strich,~,~] = fzero(@(t)fun_t(t)-(S_end.^2+w_lane.^2).^0.5,[1 4]);
        t_lc=max([t_lc t_lc_strich]);
        t_lc=0.1*round(t_lc/0.1);
        S_min_dyn=max([(max([w_lane (0-V_0.^2)/(2*a_min)]).^2-w_lane.^2).^0.5 (max([w_lane S_0+V_0*t_lc+0.5*a_min*t_lc*t_lc]).^2-w_lane.^2).^0.5]);

        index_accel_strich=max([ACC(v_max,v_e,s_e-s_d,v_d,0,CalibrationVars)/a_max_comfort 0.5]);
        V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]);
        S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc;
        V_e_end=max([0 v_e+(index_accel*a_min_comfort)*t_lc]);
        S_e_end=s_e+0.5*(V_e_end+v_e)*t_lc;
        V_d_end=max([0 v_d+(index_accel_strich*a_max_comfort)*t_lc]);
        S_d_end=s_d+0.5*(V_d_end+v_d)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        if (-s_d>v_d*t_lc) && ...,
                (s_e<=V_0*t_lc)
            %                 b车不存在 c车存在
            % V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif ( -s_d<=v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车存在 c车不存在
            % V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        elseif (-s_d>v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车不存在 c车不存在
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        else
            %                 b车存在 c车存在
            % V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]);
            V_end=((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+max([V_d_end-V_end 0])*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_min_dyn]);
            S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]);
        end
        prereq1=(S_e_end-S_d_end-l_veh-l_veh>V_d_end*t_re+(V_e_end.^2-V_d_end.^2)/(2*a_min));
        prereq2=(S_a_end>0.5*(S_0+S_end+l_veh+l_veh));
        prereq3=(V_end>V_0+a_min*t_lc);
        prereq4=(V_end<V_0+a_max*t_lc);
        prereq5=(S_max>S_end&&S_end>S_min);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh; % 距离路口过近时不允许换道
        prereq7=(s_d<=-l_veh);
        prereq8=(s_e>=l_veh);
        prereq9=(S_a_mid>0.5*S_end+l_veh);
        prereq10=(s_a>=l_veh+0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq2&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            TargetLaneIndex=BackupTargetLaneIndex;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    else
        index_accel_strich=max([ACC(v_max,v_e,s_e-s_d,v_d,0,CalibrationVars)/a_max_comfort 0.5]);
        V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]);
        S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc;
        V_e_end=max([0 v_e+(index_accel*a_min_comfort)*t_lc]);
        S_e_end=s_e+0.5*(V_e_end+v_e)*t_lc;
        V_d_end=max([0 v_d+(index_accel_strich*a_max_comfort)*t_lc]);
        S_d_end=s_d+0.5*(V_d_end+v_d)*t_lc;
        t_mid=0.5*t_lc;
        V_a_mid=max([0 v_a+(index_accel*a_min_comfort)*t_mid]);
        S_a_mid=s_a+0.5*(V_a_mid+v_a)*t_mid;
        if (-s_d>v_d*t_lc) && ...,
                (s_e<=V_0*t_lc)
            %                 b车不存在 c车存在
            V_end=min([0.5*(V_0+V_e_end) V_0]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif ( -s_d<=v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车存在 c车不存在
            V_end=min([0.5*(V_0+V_d_end) V_0]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        elseif (-s_d>v_d*t_lc) && ...,
                ( s_e>V_0*t_lc)
            %                 b车不存在 c车不存在
            S_end=V_0*t_lc;
            V_end=V_0;
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
        else
            %                 b车存在 c车存在
            V_end=min([V_0 0.5*(V_d_end+V_e_end)]);
            S_min=max([S_d_end+V_d_end*t_re+l_veh S_d_end+V_d_end*t_re+(V_end.^2-V_d_end.^2)/(2*a_min)+l_veh S_0+V_0*t_lc+0.5*a_min_comfort*t_lc*t_lc]);
            S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]);
            S_end=median([S_min S_max t_lc*0.5*(V_end+V_0)]);
        end
        S_max_withAccel=t_lc*min([V_end V_0])+0.5*a_max_comfort*t_lc*t_lc-((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移
        S_min_withAccel=t_lc*max([V_end V_0])-0.5*a_max_comfort*t_lc*t_lc+((a_max_comfort*t_lc-abs(V_end-V_0))/2).^2/(a_max_comfort); % 换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移
        prereq1=(S_e_end-S_d_end-l_veh-l_veh>V_d_end*t_re+(V_e_end.^2-V_d_end.^2)/(2*a_min));
        prereq2=(S_a_end>0.5*(S_0+S_end+l_veh+l_veh));
        prereq3=(V_end>=V_0+a_min_comfort*t_lc);
        prereq4=(V_end<=V_0+a_max_comfort*t_lc);
        prereq5=(S_max>=S_min&&S_end<=S_max_withAccel&&S_end>=S_min_withAccel);
        prereq6=d_veh2int>=S_end+indexAfterLaneChangeDis2Int*l_veh; % 距离路口过近时不允许换道
        % prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道
        prereq7=(s_d<=-l_veh);
        prereq8=(s_e>=l_veh);
        prereq9=(S_a_mid>0.5*S_end+l_veh);
        prereq10=(s_a>=l_veh+0.25*t_lc*(0.75*V_0+0.25*V_end));
        if prereq1&&prereq2&&prereq3&&prereq4&&prereq5&&prereq6&&prereq7&&prereq8&&prereq9&&prereq10
            CountLaneChange=CountLaneChange+1;
            TargetLaneIndex=BackupTargetLaneIndex;
            CurrentTargetLaneIndex=TargetLaneIndex;
        end
    end
end
% para=[];
% path generation
if TargetLaneIndex<=CurrentLaneIndex
    w_lane=w_lane_left;
else
    w_lane=w_lane_right;
end
if CountLaneChange==1
    t_lc_traj=t_lc;
    para=[3 4*S_end 5*S_end*S_end;6 12*S_end 20*S_end*S_end;S_end.^3 S_end.^4 S_end.^5]\[0;0;w_lane*10.^6];
% SpeedPlanner
    fun_S = @(x)sqrt(1+((3*para(1)*x.^2+4*para(2)*x.^3+5*para(3)*x.^4)/(10.^6)).^2);    
    x1=linspace(0,S_end,100);
    S_tlc=trapz(x1,fun_S(x1));
    para_ST=[1 0 0 0;0 1 0 0;1 t_lc t_lc.^2 t_lc.^3;0 1 2*t_lc 3*t_lc.^2]\[0;V_0;S_tlc;V_end];
% ST参数到轨迹
    for i_traj=1:1:round(t_lc/0.05)+1
        t_traj=0.05*(i_traj-1);
        S_traj(i_traj)= para_ST(1)+para_ST(2)*t_traj+para_ST(3)*t_traj.^2+para_ST(4)*t_traj.^3;
        V_traj(i_traj)=para_ST(2)+para_ST(3)*2*t_traj+para_ST(4)*3*t_traj.^2;
        fun_a = @(x)sqrt(1+((3*para(1)*x.^2+4*para(2)*x.^3+5*para(3)*x.^4)/(10.^6)).^2);
        fun_x=@(x)trapz(linspace(0,x,50),fun_a(linspace(0,x,50)));
        [X_traj(i_traj),~,~] = fzero(@(x)fun_x(x)-S_traj(i_traj),[0 S_tlc]);       
    end
    for ii=1:1:(t_lc/0.05)
        LaneChangePath(ii,1)=pos_s+X_traj(ii+1);
        LaneChangePath(ii,2)=pos_l_CurrentLane+(-1)*(double(TargetLaneIndex)-double(CurrentLaneIndex))*(para(1)*X_traj(ii+1).^3+para(2)*X_traj(ii+1).^4+para(3)*X_traj(ii+1).^5)*10.^-6;  
        LaneChangePath(ii,3)=90-(-1)*(double(TargetLaneIndex)-double(CurrentLaneIndex))*180/pi()*atan((para(1)*3*X_traj(ii+1).^2+para(2)*4*X_traj(ii+1).^3+para(3)*5*X_traj(ii+1).^4)*10.^-6);
        LaneChangePath(ii,4)=V_traj(ii+1)*cosd(180/pi()*atan((para(1)*3*X_traj(ii+1).^2+para(2)*4*X_traj(ii+1).^3+para(3)*5*X_traj(ii+1).^4)*10.^-6));
        LaneChangePath(ii,5)=(-1)*(double(TargetLaneIndex)-double(CurrentLaneIndex))*V_traj(ii+1)*sind(180/pi()*atan((para(1)*3*X_traj(ii+1).^2+para(2)*4*X_traj(ii+1).^3+para(3)*5*X_traj(ii+1).^4)*10.^-6));
        if ii==1
            LaneChangePath(ii,6)=0;
        else
            LaneChangePath(ii,6)=(LaneChangePath(ii,3)-LaneChangePath(ii-1,3))/0.05;
        end
    end
    if LaneChangePath(round(t_lc/0.05),4)==0
        LaneChangePath(round(t_lc/0.05),4)=V_traj(round(t_lc/0.05)+1);
    end
    DurationLaneChange=DurationLaneChange+1;
    CountLaneChange=CountLaneChange+1;
end

if DurationLaneChange>0 && DurationLaneChange<=round(10*t_lc_traj)
    % for count_1=1:1:2*(21-DurationLaneChange)
    for count_1=1:1:2*(round(t_lc_traj/0.1)+1-DurationLaneChange)
        traj(1,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,1);
        traj(2,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,2);
        traj(3,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,3);
        traj(4,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,4);
        traj(5,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,5);
        traj(6,count_1)=LaneChangePath(DurationLaneChange*2+count_1-2,6);
    end
    % for count_2=2*(21-DurationLaneChange)+1:1:80
    for count_2=2*(round(t_lc_traj/0.1)+1-DurationLaneChange)+1:1:120
        % traj(1,count_2)=LaneChangePath(40,1)+traj(4,2*(21-DurationLaneChange))*0.05*double(count_2-2*(21-DurationLaneChange));
        % traj(2,count_2)=LaneChangePath(40,2);
        % traj(3,count_2)=LaneChangePath(40,3); 
        % traj(4,count_2)=traj(4,2*(21-DurationLaneChange));
        traj(1,count_2)=LaneChangePath(2*(round(t_lc_traj/0.1)),1)+traj(4,2*(round(t_lc_traj/0.1)+1-DurationLaneChange))*0.05*double(count_2-2*(round(t_lc_traj/0.1)+1-DurationLaneChange));
        traj(2,count_2)=LaneChangePath(2*(round(t_lc_traj/0.1)),2);
        traj(3,count_2)=LaneChangePath(2*(round(t_lc_traj/0.1)),3);
        traj(4,count_2)=traj(4,2*(round(t_lc_traj/0.1)+1-DurationLaneChange));
        traj(5,count_2)=0;
        traj(6,count_2)=0;
    end
    DurationLaneChange=DurationLaneChange+1;
    SwitchACC=0;
elseif DurationLaneChange==0
    SwitchACC=1;
elseif DurationLaneChange> (10*t_lc_traj)
    SwitchACC=1;
    CountLaneChange=int16(0);
    DurationLaneChange=int16(0);
else
    SwitchACC=1;
end
%         ACC Function
if SwitchACC
    if CurrentLaneIndex~=TargetLaneIndex && d_veh2int<((v_max_int.^2-v_max.^2)/(2*a_min)+v_max_int*t_permit)
    % if 0
        a_soll=ACC(30/3.6,v_a,s_a,speed,wait,CalibrationVars);
    else
        a_soll=ACC(v_max,v_a,s_a,speed,wait,CalibrationVars);
    end
end
traj_s=traj(1,1:80);
traj_l=traj(2,1:80);
traj_psi=traj(3,1:80);
traj_vs=traj(4,1:80);
traj_vl=traj(5,1:80);
traj_omega=traj(6,1:80);

GlobVars.TrajPlanLaneChange.CountLaneChange=CountLaneChange;
GlobVars.TrajPlanLaneChange.DurationLaneChange=DurationLaneChange;
GlobVars.TrajPlanLaneChange.LaneChangePath=LaneChangePath;
GlobVars.TrajPlanLaneChange.t_lc_traj=t_lc_traj;
GlobVars.TrajPlanLaneChange.CurrentTargetLaneIndex=CurrentTargetLaneIndex;

% if SwitchACC
%     a_soll=ACC(v_max,v_a,s_a,speed,wait);
%     for count_1=1:1:2
%         t_count_1=0.05*count_1;
%         traj(1,count_1)=pos_s+V_0*t_count_1+0.5*a_soll*t_count_1.^2;
%         traj(2,count_1)=pos_l;
%         traj(3,count_1)=90;
%         traj(4,count_1)=V_0+a_soll*t_count_1;
%         traj(5,count_1)=0;
%         traj(6,count_1)=0;
%     end
%     jerk_2=(0-a_soll)/(4-0.1);
%     for count_2=3:1:80
%         t_count_2=0.05*(count_2-2);
%         if (a_soll*0.1+V_0)+a_soll*t_count_2+0.5*jerk_2*t_count_2.^2<=0
%             traj(1,count_2)=traj(1,count_2-1);
%         else
%             traj(1,count_2)=traj(1,2)+(a_soll*0.1+V_0)*t_count_2+0.5*a_soll*t_count_2.^2+1/6*jerk_2*t_count_2.^3;
%         end
%         traj(2,count_2)=pos_l;
%         traj(3,count_2)=90;
%         traj(4,count_2)=max([(a_soll*0.1+V_0)+a_soll*t_count_2+0.5*jerk_2*t_count_2.^2 0]);
%         traj(5,count_2)=0;
%         traj(6,count_2)=0;
%     end
%     traci.vehicle.setSpeed('S0',traj(4,2));
% end
end
% 函数调用
% postion_veh=traci.vehicle.getPosition('S0');
% CurrentLaneIndex=traci.vehicle.getLaneIndex('S0');
% CountLaneChange=0;
% DurationLaneChange=0;
% LaneChangePath=zeros([6/0.05 6]);
% TargetLaneIndex=1;
% d_veh2int=100;
% pos_s=postion_veh(2);
% pos_l=postion_veh(1);
% w_lane=3.2;
% t_lc=2;
% function [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj]=...,
%     TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,TargetLaneBehindDis,TargetLaneBehindVel,TargetLaneFrontDis,TargetLaneFrontVel,speed,pos_s,pos_l,CurrentLaneIndex,CountLaneChange,...,
%     DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,w_lane)
% if a_soll==100
%     traci.vehicle.moveToXY('S0','12', 2, traj_l(2), traj_s(2),traj_psi(2),2);
% else
%     traci.vehicle.setSpeed('S0',a_soll*0.1+speed);
%     traci.vehicle.changeLane('S0',CurrentLaneIndex,2);


% 验证横向加速度在范围内
% clear all
% i=0;
% for V_0=3:1:15
%     i=i+1;
%     tlc(i)=max([2-0.04*(V_0-15) 2]);
%     tlc(i)=min([tlc(i) 2.5]);
%     tlc(i)=ceil(tlc(i)/0.1)*0.1;
%     w_lane=3.2;
%     S_end=tlc(i)*V_0;
%     para=[3 4*S_end 5*S_end*S_end;6 12*S_end 20*S_end*S_end;S_end.^3 S_end.^4 S_end.^5]\[0;0;w_lane*10.^6];
%     aaa=0:0.1:S_end;
%     bbb=abs((6*para(1)*aaa+12*para(2)*aaa.^2+20*para(3)*aaa.^3)/(10.^6))/...,
%         (((1+(3*para(1)*aaa.^2+4*para(2)*aaa.^3+5*para(3)*aaa.^4)/(10.^6)).^2).^1.5);
%     amax(i)=V_0.^2*max(bbb);
% end
% tlc
% amax


