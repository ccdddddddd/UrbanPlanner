function [traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc]=...,
    TrajPlanLaneChange_RePlan(speed,pos_s,pos_l,pos_l_CurrentLane,CurrentLaneFrontDis,CurrentLaneFrontVel,DurationLaneChange_RePlan,LaneChangePath_RePlan,t_lc)
w_lane=3.2;
S_traj=zeros(1,2/0.05+1);
X_traj=zeros(1,2/0.05+1);
V_traj=zeros(1,2/0.05+1);
traj=zeros([6 80]);
V_0=speed;

t_re=0.5;
l_veh=5;
index_accel=0.5;
a_min_comfort=-1;
a_min=-3.5;

if DurationLaneChange_RePlan==0
    
    t_lc=max([0.4 abs(pos_l-pos_l_CurrentLane)/w_lane*2.5]);
    t_lc=min([t_lc 1.2]);
    t_lc=round(t_lc/0.1)*0.1;
                
    s_a=CurrentLaneFrontDis;
    v_a=CurrentLaneFrontVel;
    % S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc;
    V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]);
    S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc;
    V_end=min([0.5*(V_0+V_a_end) V_0]);
    V_end=max([V_end V_0+t_lc*a_min_comfort]);

    L_end=pos_l_CurrentLane-pos_l;
    % L_end=-L_end;
    S_end=min([S_a_end-t_re*V_end-l_veh S_a_end-V_end*t_re-(V_a_end.^2-V_end.^2)/(2*a_min)-l_veh sqrt((0.5*(V_0+V_end)*t_lc).^2-L_end.^2)]);
    
    para=[1 0 0 0 0 0;0 1 0 0 0 0;0 0 2 0 0 0;1 S_end S_end.^2 S_end.^3 S_end.^4 S_end.^5;0 1 2*S_end 3*S_end.^2 4*S_end.^3 5*S_end.^4;0 0 2 6*S_end 12*S_end.^2 20*S_end.^3]\...,
        [0;tand(90-90);0;L_end;0;0];
    
    fun_S = @(x)sqrt(1+((para(2)+2*para(3)*x+3*para(4)*x.^2+4*para(5)*x.^3+5*para(6)*x.^4)).^2);
    x1=linspace(0,S_end,100);
    
    S_tlc=trapz(x1,fun_S(x1));
    para_ST=[1 0 0 0;0 1 0 0;1 t_lc t_lc.^2 t_lc.^3;0 1 2*t_lc 3*t_lc.^2]\[0;V_0;S_tlc;V_end];
    
    for i_traj=1:1:t_lc/0.05+1
        t_traj=0.05*(i_traj-1);
        S_traj(i_traj)= para_ST(1)+para_ST(2)*t_traj+para_ST(3)*t_traj.^2+para_ST(4)*t_traj.^3;
        V_traj(i_traj)=para_ST(2)+para_ST(3)*2*t_traj+para_ST(4)*3*t_traj.^2;
        fun_a = @(x)sqrt(1+((para(2)+2*para(3)*x+3*para(4)*x.^2+4*para(5)*x.^3+5*para(6)*x.^4)).^2);
        fun_x=@(x)trapz(linspace(0,x,50),fun_a(linspace(0,x,50)));
        fun_x(0)-S_traj(i_traj)
        fun_x(S_tlc)-S_traj(i_traj)
        [X_traj(i_traj),~,~] = fzero(@(x)fun_x(x)-S_traj(i_traj),[-1 S_tlc+1]);
    end
    %     plot(S_traj);
    for ii=1:1:round(t_lc/0.05)
        LaneChangePath_RePlan(ii,1)=pos_s+X_traj(ii+1);
        LaneChangePath_RePlan(ii,2)=pos_l+(para(1)+para(2)*X_traj(ii+1)+para(3)*X_traj(ii+1).^2+para(4)*X_traj(ii+1).^3+para(5)*X_traj(ii+1).^4+para(6)*X_traj(ii+1).^5);
        LaneChangePath_RePlan(ii,3)=90-180/pi()*atan(para(2)+2*para(3)*X_traj(ii+1)+3*para(4)*X_traj(ii+1).^2+4*para(5)*X_traj(ii+1).^3+5*para(6)*X_traj(ii+1).^4);
        LaneChangePath_RePlan(ii,4)=V_traj(ii+1)*cosd(180/pi()*atan(para(2)+2*para(3)*X_traj(ii+1)+3*para(4)*X_traj(ii+1).^2+4*para(5)*X_traj(ii+1).^3+5*para(6)*X_traj(ii+1).^4));
        LaneChangePath_RePlan(ii,5)=V_traj(ii+1)*sind(180/pi()*atan(para(2)+2*para(3)*X_traj(ii+1)+3*para(4)*X_traj(ii+1).^2+4*para(5)*X_traj(ii+1).^3+5*para(6)*X_traj(ii+1).^4));
        if ii==1
            LaneChangePath_RePlan(ii,6)=0;
        else
            LaneChangePath_RePlan(ii,6)=(LaneChangePath_RePlan(ii,3)-LaneChangePath_RePlan(ii-1,3))/0.05;
        end
    end
    if LaneChangePath_RePlan(round(t_lc/0.05),4)==0
        LaneChangePath_RePlan(round(t_lc/0.05),4)=V_traj(round(t_lc/0.05)+1);
    end
    % plot(LaneChangePath_RePlan(1:round(t_lc/0.05),1),LaneChangePath_RePlan(1:round(t_lc/0.05),2));
    
    DurationLaneChange_RePlan=DurationLaneChange_RePlan+1;
end


if DurationLaneChange_RePlan>0 && DurationLaneChange_RePlan<=(10*t_lc)
    for count_1=1:1:2*(round(t_lc/0.1)+1-DurationLaneChange_RePlan)
        traj(1,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,1);
        traj(2,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,2);
        traj(3,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,3);
        traj(4,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,4);
        traj(5,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,5);
        traj(6,count_1)=LaneChangePath_RePlan(DurationLaneChange_RePlan*2+count_1-2,6);
    end
    for count_2=2*(round(t_lc/0.1)+1-DurationLaneChange_RePlan)+1:1:80
        %         traj(1,count_2)=LaneChangePath_2(2*(round((t_lc)/0.1)),1)+LaneChangePath_2(2*(round((t_lc)/0.1)),4)*0.05*double(count_2-2*(round(t_lc/0.1)+1-DurationLaneChange_2));
        %         traj(1,count_2)=LaneChangePath_2(2*(round((t_lc)/0.1)),1)+V_0*0.05*double(count_2-2*(round(t_lc/0.1)+1-DurationLaneChange_2));
        traj(1,count_2)=LaneChangePath_RePlan(2*(round((t_lc)/0.1)),1)+traj(4,2*(round(t_lc/0.1)+1-DurationLaneChange_RePlan))*0.05*double(count_2-2*(round(t_lc/0.1)+1-DurationLaneChange_RePlan));
        traj(2,count_2)=LaneChangePath_RePlan(2*(round((t_lc)/0.1)),2);
        traj(3,count_2)=LaneChangePath_RePlan(2*(round((t_lc)/0.1)),3);
        traj(4,count_2)=traj(4,2*(round(t_lc/0.1)+1-DurationLaneChange_RePlan));
        traj(5,count_2)=0;
        traj(6,count_2)=0;
    end
    
    %     traci.vehicle.moveToXY('S0','M0', 2, LaneChangePath(DurationLaneChange,1), LaneChangePath(DurationLaneChange,2),...,
    %         LaneChangePath(DurationLaneChange,3),2);
    % %         traci.vehicle.moveToXY('S0','M0', 2, traj(1,2), traj(2,2),traj(3,2),2);
    
    DurationLaneChange_RePlan=DurationLaneChange_RePlan+1;
end
if DurationLaneChange_RePlan>(10*t_lc)
    DurationLaneChange_RePlan=0*DurationLaneChange_RePlan;
end

traj_s=traj(1,:);
traj_l=traj(2,:);
traj_psi=traj(3,:);
traj_vs=traj(4,:);
traj_vl=traj(5,:);
traj_omega=traj(6,:);

end
