function [traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars]=...,
    TrajPlanLaneChange_RePlan(a_soll,speed,pos_s,pos_l,pos_psi,pos_l_CurrentLane,stopdistance,SampleTime,a_soll_ACC,CurrentLaneFrontVel,GlobVars,CalibrationVars,Parameter)
%globalVariable----------------------------------------------------------------------------------------------------------------------
DurationLaneChange_RePlan=GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan;
FrontWheelAnglelLimit=CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit; % 初始值15度
l_veh=Parameter.l_veh;
a_lateral=CalibrationVars.TrajPlanLaneChange.a_lateral; % 默认为4
%初值------------------------------------------------------------
    para4.form='pp';
    para4.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1;
    para4.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2;
    para4.pieces=4; 
    para4.order=4;
    para4.dim=1;
    
    para3.form='pp';
    para3.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1(1,1:4);
    para3.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2(1:3,:);
    para3.pieces=3;
    para3.order=4;
    para3.dim=1;
%----------------------------------------------------------------


% para.form='pp';
% if GlobVars.TrajPlanLaneChange_RePlan.para3==4
%     para.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1;
%     para.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2;
%     para.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3; 
% else
%     para.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1(1,1:4);
%     para.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2(1:3,:);
%     para.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3;
% end
% para.order=4;
% para.dim=1;
S_end=GlobVars.TrajPlanLaneChange_RePlan.s_end; % 初始值0
traj=zeros([6 80]);
% 过渡路径生成
if DurationLaneChange_RePlan==0
    Rdynamic=speed.^2/a_lateral;
    Rkinematic=l_veh*cotd(FrontWheelAnglelLimit);
    Rreplan=max(Rdynamic,Rkinematic);
    [CenterS,CenterL]=ReplanCenter(Rreplan,pos_s,pos_l,pos_l_CurrentLane,pos_psi);
    S_end=CenterS+sqrt(Rreplan.^2-(pos_l_CurrentLane-CenterL).^2);
%     if pos_psi~=90 && (pos_psi-90)*(pos_l-pos_l_CurrentLane)<=0 % 输入para函数的为三个点
    if abs(pos_psi-90)>=0.01 && (pos_psi-90)*(pos_l-pos_l_CurrentLane)<=0 % 输入para函数的为三个点
        Rdynamic_extre=speed.^2/(a_lateral*2.5);
        Rkinematic_extre=l_veh*cotd(FrontWheelAnglelLimit*2);
        Rreplan_extre=max(Rdynamic_extre,Rkinematic_extre);
        [CenterS_extre,CenterL_extre]=ReplanCenter(Rreplan_extre,pos_s,pos_l,pos_l_CurrentLane,pos_psi);
        [para]=Para(pos_s,pos_l,pos_psi,S_end,pos_l_CurrentLane,CenterS_extre,CenterL_extre-Rreplan_extre*sign(pos_psi-90));
    else % 输入para函数的为两个点
        [para]=Para(pos_s,pos_l,pos_psi,S_end,pos_l_CurrentLane);
    end
    for iter=1:1:5
        if iter<=length(para.breaks)
            GlobVars.TrajPlanLaneChange_RePlan.para1(iter)=para.breaks(iter);
        else
            GlobVars.TrajPlanLaneChange_RePlan.para1(iter)=0;
        end
    end
    for iter=1:1:4
        if iter<=size(para.coefs,1)
            GlobVars.TrajPlanLaneChange_RePlan.para2(iter,:)=para.coefs(iter,:);
        else
            GlobVars.TrajPlanLaneChange_RePlan.para2(iter,:)=zeros([1 4]);
        end
    end
%     GlobVars.TrajPlanLaneChange_RePlan.para1=[para.breaks zeros([1 5-length(para.breaks)])];
%     GlobVars.TrajPlanLaneChange_RePlan.para2=[para.coefs;zeros([4-size(para.coefs,1) 4])];
    GlobVars.TrajPlanLaneChange_RePlan.para3=para.pieces;
    GlobVars.TrajPlanLaneChange_RePlan.s_end=S_end;
    DurationLaneChange_RePlan=DurationLaneChange_RePlan+1;
    % 画图
%     s=linspace(pos_s,S_end,50);
%     l = ppval(para,s);
%     plot(s-pos_s,l,'r')
%     hold on
%     x1=pos_s:0.1:S_end;
%     y1=sqrt(Rreplan.^2-(x1-CenterS).^2)+CenterL;
%     plot(x1-pos_s,y1,'k--')
%     hold on
%     x2=pos_s:0.1:CenterS_extre+sqrt(Rreplan_extre.^2-(pos_l_CurrentLane-CenterL_extre).^2);
%     y2=sqrt(Rreplan_extre.^2-(x2-CenterS_extre).^2)+CenterL_extre;
%     plot(x2-pos_s,y2,'k--')
%     axis([0 S_end-pos_s+0.5 0 8])
%     axis equal
%     ax=gca;
%     ax.XAxisLocation='origin';
%     ax.YAxisLocation='origin';
%     ax.Box='off';
end

if GlobVars.TrajPlanLaneChange_RePlan.para3==4
    para4.form='pp';
    para4.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1;
    para4.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2;
    para4.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3; 
    para4.order=4;
    para4.dim=1;
else
    para3.form='pp';
    para3.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1(1,1:4);
    para3.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2(1:3,:);
    para3.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3;
    para3.order=4;
    para3.dim=1;
end

if DurationLaneChange_RePlan>0 && pos_s<S_end%生成轨迹
   %------------------------------------------------------------------------------------------------------------------------------ 
   IsStopSpeedPlan=0;
   if stopdistance<200&&speed.^2/8<=stopdistance && (-((4/9)*speed.^2/(2/3*stopdistance))<=a_soll_ACC || CurrentLaneFrontVel<0.2)
       a_soll=max(a_soll,-((4/9)*speed.^2/(2/3*stopdistance)));
       [a_soll]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll,CalibrationVars);
       if a_soll>=-((4/9)*speed.^2/(2/3*stopdistance))
           tend=(3*sqrt(max(0,(4/9)*speed.^2+(2/3)*a_soll*stopdistance))-2*speed)/(a_soll+eps);
           jerk=-2*(speed+a_soll*tend)/(tend.^2);
           for count_1=1:1:80
               t_count_1=0.05*count_1;
               if t_count_1<=tend
                   v_count_1= speed+a_soll*t_count_1+0.5*jerk*t_count_1.^2;
                   length_count_1=speed*t_count_1+0.5*a_soll*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
               else
                   v_count_1=0;
                   length_count_1=stopdistance;
               end
               if GlobVars.TrajPlanLaneChange_RePlan.para3==4
                   [traj(1,count_1),traj(2,count_1),traj(3,count_1)]=ReplanTrajPosCalc4(length_count_1,para4,pos_s,S_end,pos_l_CurrentLane);
               else
                   [traj(1,count_1),traj(2,count_1),traj(3,count_1)]=ReplanTrajPosCalc3(length_count_1,para3,pos_s,S_end,pos_l_CurrentLane);
               end
               traj(4,count_1)=v_count_1*cosd(90-traj(3,count_1));
               traj(5,count_1)=v_count_1*sind(90-traj(3,count_1));
               if count_1==1
                   traj(6,count_1)=0;
               else
                   traj(6,count_1)=(traj(3,count_1)-traj(3,count_1-1))/0.05;
               end
           end
           IsStopSpeedPlan=1;
       end
   end
   %------------------------------------------------------------------------------------------------------------------------------
   if IsStopSpeedPlan==0
       [a_soll]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll,CalibrationVars);
       for count_1=1:1:80
           t_count_1=0.05*count_1;
           v_count_1=max([0 speed+a_soll*t_count_1]);
           if v_count_1==0
               length_count_1=(0-speed.^2)/(2*a_soll+eps);
           else
               length_count_1=(v_count_1+speed)*t_count_1/2;
           end
           if GlobVars.TrajPlanLaneChange_RePlan.para3==4
               [traj(1,count_1),traj(2,count_1),traj(3,count_1)]=ReplanTrajPosCalc4(length_count_1,para4,pos_s,S_end,pos_l_CurrentLane);
           else
               [traj(1,count_1),traj(2,count_1),traj(3,count_1)]=ReplanTrajPosCalc3(length_count_1,para3,pos_s,S_end,pos_l_CurrentLane);
           end
           traj(4,count_1)=v_count_1*cosd(90-traj(3,count_1));
           traj(5,count_1)=v_count_1*sind(90-traj(3,count_1));
           if count_1==1
               traj(6,count_1)=0;
           else
               traj(6,count_1)=(traj(3,count_1)-traj(3,count_1-1))/0.05;
           end
       end
   end
    DurationLaneChange_RePlan=DurationLaneChange_RePlan+1;
end
if traj(1,2)>=S_end || pos_s>=S_end
    DurationLaneChange_RePlan=0*DurationLaneChange_RePlan;
end
traj_s=traj(1,:);
traj_l=traj(2,:);
traj_psi=traj(3,:);
traj_vs=traj(4,:);
traj_vl=traj(5,:);
traj_omega=traj(6,:);
GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan=DurationLaneChange_RePlan;
GlobVars.Decider.a_sollpre2traj=a_soll;
% t=(0.05:0.05:4);
% traj_s
% plot(t,traj_s-40);
% if GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan>0
%     figure;
%     plot(traj_s,traj_l)
%     legend;
%     
%     figure;
%     plot(traj_l)
%     legend;
%     
%     figure;
%     plot(traj_psi)
%     legend;
%     
%     figure;
%     plot(diff(traj_s)/0.05)
%     hold on;
%     plot(traj_vs)
%     legend;
%     
%     figure;
%     plot(diff(traj_l)/0.05)
%     hold on;
%     plot(traj_vl)
%     legend;
%     
%     figure;
%     plot(diff(traj_psi)/0.05)
%     hold on;
%     plot(traj_omega)
%     legend;
%     
%     figure;
%     plot(diff((traj_vl.^2+traj_vs.^2).^0.5)/0.05)
%     figure;
%     plot((traj_vl.^2+traj_vs.^2).^0.5)
% end
end
function[para]=Para(pos_s,pos_l,pos_psi,send,lend,mid_s,mid_l)%计算三次曲线参数
if nargin==7
    s=[pos_s-0.1*cosd(90-pos_psi),pos_s,mid_s,send,send+1];
    l=[pos_l-sind(90-pos_psi)*0.1,pos_l,mid_l,lend,lend];
else
    s=[pos_s-0.1*cosd(90-pos_psi),pos_s,send,send+1];
    l=[pos_l-sind(90-pos_psi)*0.1,pos_l,lend,lend];
end
paraNew=pchip(s,l);
para.breaks=paraNew.breaks;
para.coefs=paraNew.coefs;
para.pieces=size(paraNew.coefs,1);
para.order=4;
end
function [s,l,psi]=ReplanTrajPosCalc4(length,para4,s0,send,lend)%计算沿曲线给定长度的点位置
% fprime = fnder(para,1);
%fprime = fnder(para,1);---------------------------------------------------
vector=(4-1:-1:1);
dcoefs_old=para4.coefs*diag(vector,1);
dcoefs=dcoefs_old(:,2:end);
% fprime=para;
% fprime.coefs=dcoefs;
% fprime.order=para.order-1;
fprime=mkpp(para4.breaks,dcoefs);
%--------------------------------------------------------------------------
fun_a = @(x)sqrt(1+(ppval(fprime,x)).^2);
fun_x=@(x)trapz(linspace(s0,x,50),fun_a(linspace(s0,x,50)));
if fun_x(send)>=length
    [s,~,~] = fzero(@(x)fun_x(x)-length,[s0 send]);
    para4=mkpp(para4.breaks,para4.coefs);
    l = ppval(para4,s);
    psi=90-atand(ppval(fprime,s));
else
    s= length-fun_x(send)+send;
    l=lend;
    psi=90;
end
end
function [s,l,psi]=ReplanTrajPosCalc3(length,para3,s0,send,lend)%计算沿曲线给定长度的点位置
% fprime = fnder(para,1);
%fprime = fnder(para,1);---------------------------------------------------
vector=(4-1:-1:1);
dcoefs_old=para3.coefs*diag(vector,1);
dcoefs=dcoefs_old(:,2:end);
% fprime=para;
% fprime.coefs=dcoefs;
% fprime.order=para.order-1;
fprime=mkpp(para3.breaks,dcoefs);
%--------------------------------------------------------------------------
fun_a = @(x)sqrt(1+(ppval(fprime,x)).^2);
fun_x=@(x)trapz(linspace(s0,x,50),fun_a(linspace(s0,x,50)));
if fun_x(send)>=length
    [s,~,~] = fzero(@(x)fun_x(x)-length,[s0 send]);
    para3=mkpp(para3.breaks,para3.coefs);
    l = ppval(para3,s);
    psi=90-atand(ppval(fprime,s));
else
    s= length-fun_x(send)+send;
    l=lend;
    psi=90;
end
end
function [CenterS,CenterL]=ReplanCenter(Rreplan,pos_s,pos_l,pos_l_CurrentLane,pos_psi)
if (sign(pos_l-pos_l_CurrentLane)==1&&pos_psi<=90)
    CenterS=pos_s+Rreplan*cosd(pos_psi);
    CenterL=pos_l-Rreplan*sind(pos_psi);
elseif sign(pos_l-pos_l_CurrentLane)==1&&pos_psi>90
    CenterS=pos_s+Rreplan*cosd(180-pos_psi);
    CenterL=pos_l-Rreplan*sind(180-pos_psi);
elseif sign(pos_l-pos_l_CurrentLane)==-1&&pos_psi<=90
    CenterS=pos_s+Rreplan*cosd(pos_psi);
    CenterL=pos_l+Rreplan*sind(pos_psi);
elseif sign(pos_l-pos_l_CurrentLane)==-1&&pos_psi>90
    CenterS=pos_s+Rreplan*cosd(180-pos_psi);
    CenterL=pos_l+Rreplan*sind(180-pos_psi);
else
    if pos_psi<90
        CenterS=pos_s+Rreplan*cosd(pos_psi);
        CenterL=pos_l-Rreplan*sind(pos_psi);
    else
        CenterS=pos_s+Rreplan*cosd(180-pos_psi);
        CenterL=pos_l+Rreplan*sind(180-pos_psi);
    end 
end
end