function [a_soll_TrajPlanTurnAround,a_sollTurnAround2Decider,Refline,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,GlobVars,TargetGear,TurnAroundActive,AEBactive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLanes,s_turnaround_border,...,
    IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,LengthOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,LengthCodirectCar,CurrentLane,v_max,a_soll,...,
    CurrentGear,TurnAroundActive,AEBactive,stopdistance,a_soll_ACC,SampleTime,GlobVars,CalibrationVars,Parameters)
%--------------------------------------------------------------------------------------------------------------------------------------------------------
%Parameters
TurningRadius=Parameters.turningRadius;
w_veh=Parameters.w_veh;
l_veh=Parameters.l_veh;
%---------------------------------------------------------------------------------------------------------------------------------------------------------
%CalibrationVariable
% D_safe1=0.5; %D_safeSboder2HostVeh
% D_safe2=2;   %D_safeEvVeh2HostVeh
% dec2line=0.2;
% a_min=-2;
% a_max_com=1.5;
% v_max_turnAround=5;
D_safe1=CalibrationVars.TrajPlanTurnAround.d_safe1;
D_safe2=CalibrationVars.TrajPlanTurnAround.d_safe2;
dec2line=CalibrationVars.TrajPlanTurnAround.dec2line;
a_min=CalibrationVars.TrajPlanTurnAround.a_min;
a_max_com=CalibrationVars.TrajPlanTurnAround.a_max_com;
v_max_turnAround=CalibrationVars.TrajPlanTurnAround.v_max_turnAround;
d_gap2stop=CalibrationVars.TrajPlanTurnAround.d_gap2stop;
%---------------------------------------------------------------------------------------------------------------------------------------------------------
%globalVariable
PosCircle1=GlobVars.TrajPlanTurnAround.posCircle;
PosCircle2=GlobVars.TrajPlanTurnAround.posCircle2;
PosCircle3=GlobVars.TrajPlanTurnAround.posCircle3;
pos_start=GlobVars.TrajPlanTurnAround.pos_start;
pos_mid1=GlobVars.TrajPlanTurnAround.pos_mid1;
pos_mid2=GlobVars.TrajPlanTurnAround.pos_mid2;
pos_mid1_rear=GlobVars.TrajPlanTurnAround.pos_mid1_rear;
pos_mid2_rear=GlobVars.TrajPlanTurnAround.pos_mid2_rear;
pos_end=GlobVars.TrajPlanTurnAround.pos_end;
LaneCenterline=GlobVars.TrajPlanTurnAround.laneCenterline;
dec_trunAround=GlobVars.TrajPlanTurnAround.dec_trunAround;
wait_turnAround=GlobVars.TrajPlanTurnAround.wait_turnAround;
TypeOfTurnAround=GlobVars.TrajPlanTurnAround.typeOfTurnAround;
TurnAroundState=GlobVars.TrajPlanTurnAround.turnAroundState;
TargetLaneIndexOpposite=GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite;
%---------------------------------------------------------------------------------------------------------------------------------------------------------
%intermediateVars
d_veh2cross=zeros([5,1]);
traj_s=zeros([1 80]);
traj_l=zeros([1 80]);
traj_psi=zeros([1 80]);
traj_vs=zeros([1 80]);
traj_vl=zeros([1 80]);
traj_omega=zeros([1 80]);
Refline.NumRefLaneTurnAround=int16(0);
Refline.SRefLaneTurnAround=zeros([1 100],'double');
Refline.LRefLaneTurnAround=zeros([1 100],'double');
Refline.TurnAroundReflineState=int16(0);
TargetGear=CurrentGear;
IndexOfLaneOppositeCarFront=zeros([6,1]);
SpeedOppositeCarFront=zeros([6,1]);
PosSOppositeCarFront=zeros([6,1]);
PosSOppositeCarFront=PosSOppositeCarFront+200;
SpeedOppositeCarRear=zeros([6,1]);
PosSOppositeCarRear=zeros([6,1]);
PosSOppositeCarRear=PosSOppositeCarRear-200;
LengthOppositeCarFront=zeros([6,1])+5;
LengthOppositeCarRear=zeros([6,1])+5;
dis2pos_mid2=200;

wait_TrafficLight=GlobVars.SpeedPlanTrafficLight.wait_TrafficLight;
% 目标车道选择 ----(判断掉头一次二次，选择掉头目标车道，只计算一次)
pos_l_TargetLane=0;%20220324因为只用一次，所以可以设初值
WidthOfLaneCurrent=WidthOfLanes(CurrentLane);
if TypeOfTurnAround==0
    % d_cur2tar=0.5*(WidthOfLanesOpposite(1)-w_veh)+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(1);
    TargetLaneIndexOpposite=int16(1);
    TypeOfTurnAround=int16(2); % 1为一次顺车掉头，2为二次顺车掉头
    for i=1:NumOfLanesOpposite
        d_cur2pot_tar=0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i);
        if i>1
            for j=1:i-1
                d_cur2pot_tar=d_cur2pot_tar+WidthOfLanesOpposite(j);
            end
        end
        if d_cur2pot_tar>=2*TurningRadius
            d_cur2tar=d_cur2pot_tar;
            TargetLaneIndexOpposite=i;
            TypeOfTurnAround=int16(1);
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        elseif 0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar>=2*TurningRadius
            % d_cur2tar=0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar;
            d_cur2tar=2*TurningRadius;
            TargetLaneIndexOpposite=i;
            TypeOfTurnAround=int16(1);
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        end
    end
    % 一次顺车掉头路径生成
    if TypeOfTurnAround==1
        PosCircle1(1)=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe1;
        PosCircle1(2)=pos_l_CurrentLane+TurningRadius;
        % s_start=PosCircle1(1);
        PosCircle2(1)=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe1;
        PosCircle2(2)=pos_l_TargetLane-TurningRadius;
        LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite); % 车道中心线位置 全局变量
        %掉头参考线输出及参考线过渡结束位置的坐标
        [NumRefLaneTurnAround,SRefLaneTurnAround,LRefLaneTurnAround,SEnd]=PathPlanTurnAroundDecider(LaneCenterline(TargetLaneIndexOpposite),PosCircle1,PosCircle2,TurningRadius,pos_s);
        Refline.NumRefLaneTurnAround=int16(NumRefLaneTurnAround);
        for iter=1:1:min(int16(100),NumRefLaneTurnAround)
            Refline.SRefLaneTurnAround(iter)=SRefLaneTurnAround(iter);
            Refline.LRefLaneTurnAround(iter)=LRefLaneTurnAround(iter);
        end
        GlobVars.TrajPlanTurnAround.reflineSend=SEnd;
        GlobVars.TrajPlanTurnAround.reflineLend=LaneCenterline(TargetLaneIndexOpposite);
    end
    % 二次顺车掉头路径生成
    if TypeOfTurnAround==2
        pos_l_TargetLane=pos_l_CurrentLane+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(NumOfLanesOpposite)+...,
            sum(WidthOfLanesOpposite(1:NumOfLanesOpposite-1))+0.5*(WidthOfLanesOpposite(NumOfLanesOpposite)-w_veh)-dec2line;
        TargetLaneIndexOpposite=NumOfLanesOpposite;
        pos_l_TargetLaneBoundary=pos_l_CurrentLane+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(NumOfLanesOpposite)+sum(WidthOfLanesOpposite(1:NumOfLanesOpposite-1));
        [PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_end]=PathPlanTurnAround(s_turnaround_border,w_veh,TurningRadius,D_safe1,pos_l_CurrentLane,pos_l_TargetLane,pos_l_TargetLaneBoundary,dec2line);
        pos_mid1_rear=[pos_mid1(1)+sin(pos_mid1(3))*l_veh pos_mid1(2)-cos(pos_mid1(3))*l_veh 0 0];
        pos_mid2_rear=[pos_mid2(1)+sin(pos_mid2(3))*l_veh pos_mid2(2)-cos(pos_mid2(3))*l_veh 0 0];
        % OccupiedLanesPosMid1=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1_rear(2)):1:...,
        %     LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1(2)); % 例如[2 1] 全局变量 对向车道序号为正，最左侧为1
        pos_mid1_rear(4)=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1_rear(2));
        pos_mid1(4)=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1(2));
        % OccupiedLanesPosMid2=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2_rear(2)):1:...,
        %     LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2(2)); % 例如[1 -1] 全局变量 掉头前道路车道序号为负，最左侧为-1
        pos_mid2_rear(4)=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2_rear(2));
        pos_mid2(4)=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2(2));
        LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite); % 车道中心线位置 全局变量
    end   
end
%筛选二次/一次顺车掉头前车后车到路径圆心距离
posOfLaneCenterline=zeros([6,1]);%20220324
if TypeOfTurnAround==1
%     posOfLaneCenterline=zeros([6,1]);%车道中心线在掉头路径上交点s坐标
    for i=1:length(LaneCenterline)
        if LaneCenterline(i)~=0&&i<=TargetLaneIndexOpposite
            if LaneCenterline(i) < PosCircle1(2)
                posOfLaneCenterline(i)=sqrt(TurningRadius^2-(PosCircle1(2)-LaneCenterline(i))^2)+PosCircle1(1);
            elseif LaneCenterline(i) >= PosCircle1(2) && LaneCenterline(i) <= PosCircle2(2)
                posOfLaneCenterline(i)=TurningRadius+PosCircle1(1);
            elseif LaneCenterline(i) > PosCircle2(2)
                posOfLaneCenterline(i)=sqrt(TurningRadius^2-(PosCircle2(2)-LaneCenterline(i))^2)+PosCircle2(1);
            end
        end
    end
elseif TypeOfTurnAround==2 && TurnAroundState<2
    posOfLaneCenterline=zeros([6,1]);
    for i=1:length(LaneCenterline)
        if LaneCenterline(i)~=0&&i<=TargetLaneIndexOpposite
            posOfLaneCenterline(i)=sqrt(TurningRadius^2-(PosCircle1(2)-LaneCenterline(i))^2)+PosCircle1(1);
        end
    end
end
if TypeOfTurnAround==1 ||(TypeOfTurnAround==2 && TurnAroundState<2)
    j=1;
    for i=1:length(IndexOfLaneOppositeCar)
        if IndexOfLaneOppositeCar(i)~=0&&IndexOfLaneOppositeCar(i)<=TargetLaneIndexOpposite
            if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i)) >= 0
                IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                PosSOppositeCarFront(j)=PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                SpeedOppositeCarFront(j)=SpeedOppositeCar(i);
                LengthOppositeCarFront(j)=LengthOppositeCar(i);
                j=j+1;
            end
        end
    end
    for  i=1:length(IndexOfLaneOppositeCar)
        if IndexOfLaneOppositeCar(i)~=0&&IndexOfLaneOppositeCar(i)<=TargetLaneIndexOpposite
            if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i))<0&&PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i))>PosSOppositeCarRear(IndexOfLaneOppositeCar(i))
                PosSOppositeCarRear(IndexOfLaneOppositeCar(i))=PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                SpeedOppositeCarRear(IndexOfLaneOppositeCar(i))=SpeedOppositeCar(i);
                LengthOppositeCarRear(IndexOfLaneOppositeCar(i))=LengthOppositeCar(i);
            end
        end
    end
end
% 一次顺车掉头决策
if TypeOfTurnAround==1 %&& pos_s<PosCircle1(1)
    for i=1:TargetLaneIndexOpposite
        if 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))<TurningRadius
            d_veh2cross(i)=PosCircle1(1)-pos_s+TurningRadius*acos((TurningRadius-(0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))))/TurningRadius);
        elseif 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))>TurningRadius+PosCircle2(2)-PosCircle1(2)
            d_veh2cross(i)=PosCircle1(1)-pos_s+(TurningRadius*asin((0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))...,
                -TurningRadius-PosCircle2(2)+PosCircle1(2))/TurningRadius)+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2);
        else
            d_veh2cross(i)=PosCircle1(1)-pos_s+TurningRadius*pi/2+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))-TurningRadius;
        end
        %         d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确
    end
    % 策略模式判断
    d_bre=(0-speed.^2)/(2*a_min);
    if dec_trunAround==0
        if PosCircle1(1)-pos_s<=d_bre+2*l_veh && PosCircle1(1)-pos_s>0 && pos_l<PosCircle1(2)
            dec_trunAround=int16(1);
        end
    else
        if PosCircle1(1)-pos_s<=0 || wait_turnAround==1 || pos_l>PosCircle1(2)%20220225
            dec_trunAround=int16(0);
        end
    end
    % 停车决策
    if dec_trunAround==1
        %         for i=1:TargetLaneIndexOpposite
        %             d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确
        %         end
        for j=1:length(IndexOfLaneOppositeCarFront)
            if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                d_veh=max([(d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh)/max([speed 0.00001])*SpeedOppositeCarFront(j)+0.5*w_veh+D_safe2 0]);
                if PosSOppositeCarFront(j)<=d_veh
                    wait_turnAround=int16(1);
                    break;
                end
            end
        end
        if wait_turnAround==0
            for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
%                 if PosSOppositeCarRear(j)>-l_veh
                if PosSOppositeCarRear(j)>-LengthOppositeCarRear(j)
                    wait_turnAround=int16(1);
                    break;
                end
            end
        end
    end
    % 起步决策
    if wait_turnAround==1 && PosCircle1(1)-pos_s<10
        wait_turnAround=int16(0);
        a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0,CalibrationVars);%自车预估加速度
        for j=1:length(IndexOfLaneOppositeCarFront)
            if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)/max([SpeedOppositeCarFront(j) 0.00001])]);
                % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh-TurningRadius)/max([SpeedOppositeCarFront(j) 0.00001])]);
                a_OppositeCarFront=max([ACC(50/3.6,SpeedOppositeCarRear(IndexOfLaneOppositeCarFront(j)),PosSOppositeCarFront(j)-PosSOppositeCarRear(IndexOfLaneOppositeCarFront(j)),SpeedOppositeCarFront(j),0,CalibrationVars) 0]);
                if SpeedOppositeCarFront(j)<=0.01
                    a_OppositeCarFront=0;
                end
                a_OppositeCarFront=min(a_OppositeCarFront,1.5);
                % [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-(max(SpeedOppositeCarFront(j)+a_OppositeCarFront*t-v_max,0))^2/(2*a_OppositeCarFront+eps)...,
                %     -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),...,
                %     [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]);
                [timeGap,~,~] = fzero(@(t)(max((SpeedOppositeCarFront(j)+min(SpeedOppositeCarFront(j)+(a_OppositeCarFront)*t,v_max))/2,0.0001)*t...,
                    -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),...,
                    [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 min(SpeedOppositeCarFront(j),v_max)])]);
                % s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                a_predict=min([a_predict a_max_com]);
                if a_predict>0
                    s_max=0.5*(speed+a_predict*timeGap+speed)*timeGap-(max(speed+a_predict*timeGap-v_max_turnAround,0))^2/(2*a_predict+eps);
                else
                    s_max=0.5*(median([0 speed+a_predict*timeGap v_max_turnAround])+speed)*timeGap;
                end
                if s_max<=d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh
                    wait_turnAround=int16(1);
                    break;
                end
            end
        end
        if wait_turnAround==0
            for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
%                 if PosSOppositeCarRear(j)>-l_veh
                if PosSOppositeCarRear(j)>-LengthOppositeCarRear(j)
                    wait_turnAround=int16(1);
                    break;
                end
            end
        end
    end
end
% 二次顺车掉头决策
if TypeOfTurnAround==2
    % 二次顺车掉头状态判断
    s_start=pos_start(1);
    l_start=pos_start(2);
    s_end=pos_end(1);
    l_end=pos_end(2);
    s_p1=pos_mid1(1);
    l_p1=pos_mid1(2);
    s_p2=pos_mid2(1);
    l_p2=pos_mid2(2);
    if TurnAroundState==0&&pos_s>=s_start&&pos_l<l_start+0.5&&pos_l>l_start-0.5
        TurnAroundState=int16(1);
        TargetGear=int16(4);% P R N D /1 2 3 4
    end
    if TurnAroundState==1
        r1=sqrt((s_p1-pos_s)^2+(l_p1-pos_l)^2);
        if r1<0.15&&speed<=0.05
            TargetGear=int16(2);
            if CurrentGear==2 % 环境车允许倒车 % 倒车前决策
                % 当前位置mid1车头车尾所在车道 → 确定已占据车道
                OccupiedLanesPosMid2=pos_mid2_rear(4):(pos_mid2(4)-pos_mid2_rear(4)>=0)*2-1:pos_mid2(4); % 例如[1 -1] 掉头前道路车道序号为负，最左侧为-1
                OccupiedLanesPosMid1=pos_mid1_rear(4):(pos_mid1(4)-pos_mid1_rear(4)>=0)*2-1:pos_mid1(4); % 例如[2 1] 对向车道序号为正，最左侧为1
                OccupiedLanes=OccupiedLanesPosMid2(1):1:OccupiedLanesPosMid1(length(OccupiedLanesPosMid1));
                % 目标位置mid2车尾所在车道 → 确定将侵入的车道
                [~, ia] = setdiff( OccupiedLanes,OccupiedLanesPosMid1);
                Lanes2Search = OccupiedLanes(sort(ia)); % （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道）
                Lanes2Search=Lanes2Search(Lanes2Search~=0);
                % （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将倒车路径简化为pos_mid1_rear到pos_mid2_rear的线段→d_veh2cross,timegap”
                TurnAroundState=int16(2);
                a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0,CalibrationVars);
                for i=1:length(IndexOfLaneOppositeCar)
                    if ismember(IndexOfLaneOppositeCar(i),Lanes2Search)
                        k=(pos_mid1_rear(1,2)-pos_mid2_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                        b=(pos_mid1_rear(1,1)*pos_mid2_rear(1,2)-pos_mid2_rear(1,1)*pos_mid1_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                        disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                        if disOppositeCar2circle2>0
                            timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-D_safe2)/max([SpeedOppositeCar(i) 0.00001])]);
                            s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                            d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(IndexOfLaneOppositeCar(i))).^2);
                            if s_max<=d_veh2cross_strich+l_veh
                                TurnAroundState=int16(1);
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
%                             if disOppositeCar2circle2>-l_veh
                            if disOppositeCar2circle2>-LengthOppositeCar(i)
                                TurnAroundState=int16(1);
                                break;
                            end
                        end
                    end
                end
                if TurnAroundState==2
%                     SpeedCodirectCar=SpeedCodirectCar(SpeedCodirectCar>-1);%20220324
% SpeedCodirectCar默认值为-1, LaneCenterline最后一列为掉头前所在车道
%                     for j=1:length(SpeedCodirectCar)
                     for j=1:length(SpeedCodirectCar(SpeedCodirectCar>-1))
                        if ismember(IndexOfLaneCodirectCar(j),Lanes2Search)
                            k=(pos_mid1_rear(1,2)-pos_mid2_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                            b=(pos_mid1_rear(1,1)*pos_mid2_rear(1,2)-pos_mid2_rear(1,1)*pos_mid1_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                            if IndexOfLaneCodirectCar(j)==-1
                                disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-b)/k-PosSCodirectCar(j);
                            else
                                disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-3.2-b)/k-PosSCodirectCar(j);
                            end
                            if disOppositeCar2circle2>0
                                timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-D_safe2)/max([SpeedCodirectCar(j) 0.00001])]);
                                s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                                if IndexOfLaneCodirectCar(j)==-1
                                    d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(length(LaneCenterline))-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(length(LaneCenterline))).^2);
                                else
                                    d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(length(LaneCenterline))-3.2-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(length(LaneCenterline))+3.2).^2);
                                end
                                if s_max<=d_veh2cross_strich+l_veh
                                    TurnAroundState=int16(1);
                                    break;
                                end
                            elseif disOppositeCar2circle2<=0
%                                 if disOppositeCar2circle2>-l_veh
                                if disOppositeCar2circle2>-LengthCodirectCar(j)
                                    TurnAroundState=int16(1);
                                    break;
                                end
                            end
                        end
                    end
                end
            else
                TurnAroundState=int16(1);
            end
        end
    elseif TurnAroundState==2
        r2=sqrt((s_p2-pos_s)^2+(l_p2-pos_l)^2);
        if r2<0.15&&speed<=0.05
            TargetGear=int16(4);
        end
        if r2<0.15&&CurrentGear==4&&speed<=0.05
            % 环境车允许前进 % 前进前决策
            % 当前位置mid2车头车尾所在车道 → 确定已占据车道
            OccupiedLanesPosMid2=pos_mid2_rear(4):(pos_mid2(4)-pos_mid2_rear(4)>=0)*2-1:pos_mid2(4); % 例如[1 -1] 掉头前道路车道序号为负，最左侧为-1
            OccupiedLanes=OccupiedLanesPosMid2(1):1:TargetLaneIndexOpposite;
            [~, ia] = setdiff(OccupiedLanes,OccupiedLanesPosMid2);
            Lanes2Search = OccupiedLanes(sort(ia)); % （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道）
            Lanes2Search=Lanes2Search(Lanes2Search~=0);
            % （非已占据车道的目标车道+已占据车道到目标车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将前进路径简化为pos_mid2到pos_end的线段→d_veh2cross,timegap”
            TurnAroundState=int16(3);
            a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0,CalibrationVars);
            for i=1:length(IndexOfLaneOppositeCar)
                if ismember(IndexOfLaneOppositeCar(i),Lanes2Search)
                    k=(pos_mid2(1,2)-pos_end(1,2))/(pos_mid2(1,1)-pos_end(1,1));
                    b=(pos_mid2(1,1)*pos_end(1,2)-pos_end(1,1)*pos_mid2(1,2))/(pos_mid2(1,1)-pos_end(1,1));
                    disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                    if disOppositeCar2circle2>0
                        % a_OppositeCarFront=min(a_OppositeCarFront,1.5);
                        % [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])),...,
                        %     [0-0.01 0.01+max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]);
                        timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-D_safe2)/max([SpeedOppositeCar(i) 0.00001])]);
                        s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                        d_veh2cross_strich=sqrt((pos_mid2(1)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k).^2+(pos_mid2(2)-LaneCenterline(IndexOfLaneOppositeCar(i))).^2);
                        if s_max<=d_veh2cross_strich+l_veh
                            TurnAroundState=int16(2);
                            break;
                        end
                    elseif disOppositeCar2circle2<=0
%                         if disOppositeCar2circle2>-l_veh
                        if disOppositeCar2circle2>-LengthOppositeCar(i)
                            TurnAroundState=int16(2);
                            break;
                        end
                    end
                end
            end
        else
            TurnAroundState=int16(2);
        end
    elseif TurnAroundState==3
        TargetGear=int16(4);
        if pos_s<s_end-4&&pos_l<l_end+0.5&&pos_l>l_end-0.5
            % TurnAroundState=0;
            TurnAroundActive=int16(0);
        end
    else
        TargetGear=int16(4);
    end
    % 进入掉头路径前决策
    if TurnAroundState==0 || (TurnAroundState==1 && pos_start(1)-pos_s>=0)
        for i=1:TargetLaneIndexOpposite
            if 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))<TurningRadius
                d_veh2cross(i)=PosCircle1(1)-pos_s+TurningRadius*acos((TurningRadius-(0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))))/TurningRadius);
            else
                d_veh2cross(i)=PosCircle1(1)-pos_s+(TurningRadius*asin((0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+...,
                    sum(WidthOfLanesOpposite(1:i-1))-TurningRadius)/TurningRadius)+TurningRadius*pi/2);
            end
        end
        % 策略模式判断
        d_bre=(0-speed.^2)/(2*a_min);
        if dec_trunAround==0
            if pos_start(1)-pos_s<=d_bre+2*l_veh && pos_start(1)-pos_s>0 && pos_l<PosCircle1(2) && TurnAroundState<2
                dec_trunAround=int16(1);
            end
        else
            if (pos_start(1)-pos_s<=0 || wait_turnAround==1) && pos_l<PosCircle1(2) && TurnAroundState<2
                dec_trunAround=int16(0);
            end
        end
        % 停车决策
        if dec_trunAround==1
            for j=1:length(IndexOfLaneOppositeCarFront)
                if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                    d_veh=max([(d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh)/max([speed 0.00001])*SpeedOppositeCarFront(j)+0.5*w_veh+D_safe2 0]);
                    if PosSOppositeCarFront(j)<=d_veh
                        wait_turnAround=int16(1);
                        break;
                    end
                end
            end
            if wait_turnAround==0
                for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
%                     if PosSOppositeCarRear(j)>-l_veh
                    if PosSOppositeCarRear(j)>-LengthOppositeCarRear(j)
                        wait_turnAround=int16(1);
                        break;
                    end
                end
            end
        end
        % 起步决策
        if wait_turnAround==1 && pos_start(1)-pos_s<10
            wait_turnAround=int16(0);
            % a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
            dis2pos_mid1=abs(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_l-PosCircle1(2)),(pos_s-PosCircle1(1))))*TurningRadius;
            a_predict=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars) ACC(v_max_turnAround,0,dis2pos_mid1+CalibrationVars.ACC.d_wait,speed,wait_turnAround,CalibrationVars)]);
            for j=1:length(IndexOfLaneOppositeCarFront)
                if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                    a_OppositeCarFront=max([ACC(50/3.6,SpeedOppositeCarRear(IndexOfLaneOppositeCarFront(j)),PosSOppositeCarFront(j)-PosSOppositeCarRear(IndexOfLaneOppositeCarFront(j)),SpeedOppositeCarFront(j),0,CalibrationVars) 0]);
                    if SpeedOppositeCarFront(j)<=0.01
                         a_OppositeCarFront=0;
                    end
                    a_OppositeCarFront=min(a_OppositeCarFront,1.5);
%                     [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),...,
%                         [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]);
%                     % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)/max([SpeedOppositeCarFront(j) 0.00001])]);
%                     s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                    [timeGap,~,~] = fzero(@(t)(max((SpeedOppositeCarFront(j)+min(SpeedOppositeCarFront(j)+(a_OppositeCarFront)*t,v_max))/2,0.0001)*t...,
                        -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),...,
                        [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 min(SpeedOppositeCarFront(j),v_max)])]);
                    % s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                    a_predict=min([a_predict a_max_com]);
                    if a_predict>0
                        s_max=0.5*(speed+a_predict*timeGap+speed)*timeGap-(max(speed+a_predict*timeGap-v_max_turnAround,0))^2/(2*a_predict+eps);
                    else
                        s_max=0.5*(median([0 speed+a_predict*timeGap v_max_turnAround])+speed)*timeGap;
                    end
                    if s_max<=d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh
                        wait_turnAround=int16(1);
                        break;
                    end
                end
            end
            if wait_turnAround==0
                for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
%                     if PosSOppositeCarRear(j)>-l_veh
                    if PosSOppositeCarRear(j)>-LengthOppositeCarRear(j)
                        wait_turnAround=int16(1);
                        break;
                    end
                end
            end
        end
    end
end

% ACC速度规划
a_soll_TrajPlanTurnAround=0;%20220322
if TypeOfTurnAround==1
    % 一次顺车掉头速度规划
    if (wait_turnAround==1 && PosCircle1(1)>pos_s) || wait_TrafficLight==1
        % a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]);
        a_soll_TrajPlanTurnAround=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars) ACC(v_max_turnAround,0,max([0 PosCircle1(1)-pos_s+CalibrationVars.ACC.d_wait-d_gap2stop]),speed,wait_turnAround,CalibrationVars)]);
    else
        if dec_trunAround==1
            a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
        else
            if PosCircle1(1)-pos_s>2*l_veh
                a_soll_TrajPlanTurnAround=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
            else
                a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
            end
        end
    end
    a_soll_TrajPlanTurnAround=min([a_soll_TrajPlanTurnAround,a_soll]);
elseif TypeOfTurnAround==2
    % 二次顺车掉头速度规划
    %     if TurnAroundState==0
    if TurnAroundState==0
        if wait_turnAround==1 || wait_TrafficLight==1
            a_soll_TrajPlanTurnAround=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars) ACC(v_max_turnAround,0,max([0 pos_start(1)-pos_s+CalibrationVars.ACC.d_wait-d_gap2stop]),speed,wait_turnAround,CalibrationVars)]);
        else
            if dec_trunAround==1
                a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
                dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_start(2)-PosCircle1(2)),(pos_start(1)-PosCircle1(1))))*TurningRadius+pos_start(1)-pos_s;
                a_soll_TrajPlanTurnAround=min(ACC(v_max,0,dis2pos_mid1+CalibrationVars.ACC.d_wait,speed,0,CalibrationVars),a_soll_TrajPlanTurnAround);
            else
                if pos_start(1)-pos_s>2*l_veh
                    a_soll_TrajPlanTurnAround=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
                else
                    a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
                    dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_start(2)-PosCircle1(2)),(pos_start(1)-PosCircle1(1))))*TurningRadius+pos_start(1)-pos_s;
                    a_soll_TrajPlanTurnAround=min(ACC(v_max,0,dis2pos_mid1+CalibrationVars.ACC.d_wait,speed,0,CalibrationVars),a_soll_TrajPlanTurnAround);
                end
            end
        end

    elseif TurnAroundState==1
        dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_l-PosCircle1(2)),(pos_s-PosCircle1(1))))*TurningRadius;
        a_soll_TrajPlanTurnAround=min([ACClowSpeed(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,CalibrationVars) ACClowSpeed(v_max_turnAround,0,dis2pos_mid1+CalibrationVars.ACClowSpeed.d_wait,speed,CalibrationVars)]);
        if sqrt((pos_mid1(1)-pos_s)^2+(pos_mid1(2)-pos_l)^2)<0.15
            a_soll_TrajPlanTurnAround=-4*sign(speed);
        end
    elseif TurnAroundState==2
        dis2pos_mid2=mod(atan2((pos_mid2(2)-PosCircle2(2)),(pos_mid2(1)-PosCircle2(1)))-atan2((pos_l-PosCircle2(2)),(pos_s-PosCircle2(1))),2*pi)*TurningRadius;
        a_soll_TrajPlanTurnAround=min([ACClowSpeed(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,CalibrationVars) ACClowSpeed(v_max_turnAround,0,dis2pos_mid2+CalibrationVars.ACClowSpeed.d_wait,speed,CalibrationVars)]);
        if sqrt((pos_mid2(1)-pos_s)^2+(pos_mid2(2)-pos_l)^2)<0.15
            a_soll_TrajPlanTurnAround=-4*sign(speed);
        end
    elseif TurnAroundState==3
        a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround,CalibrationVars);
    end
    a_soll_TrajPlanTurnAround=min([a_soll_TrajPlanTurnAround,a_soll]);
end

% AEB决策
if AEBactive==0
    if TypeOfTurnAround==1&&PosCircle1(1)-pos_s<=0
        for j=1:length(IndexOfLaneOppositeCar)
            % 用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离)
            if IndexOfLaneOppositeCar(j)~=0
                if (LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))<TurningRadius
                    pos_s_lane=PosCircle1(1)+TurningRadius*acos((TurningRadius-(LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius)))/TurningRadius);
                elseif (LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))>TurningRadius+PosCircle2(2)-PosCircle1(2)
                    pos_s_lane=PosCircle1(1)+(TurningRadius*asin(((LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))...,
                        -TurningRadius-PosCircle2(2)+PosCircle1(2))/TurningRadius)+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2);
                else
                    pos_s_lane=PosCircle1(1)+TurningRadius*pi/2+(LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))-TurningRadius;
                end
                if pos_l<PosCircle1(2) && pos_s<=PosCircle1(1)
                    pos_s_car=pos_s;
                elseif pos_l<PosCircle1(2) && pos_s>PosCircle1(1)
                    pos_s_car=PosCircle1(1)+atan((pos_s-PosCircle1(1))/(eps+PosCircle1(2)-pos_l))*TurningRadius;
                elseif pos_l>=PosCircle1(2) && pos_l<=PosCircle2(2)
                    pos_s_car=PosCircle1(1)+pi/2*TurningRadius+pos_l-PosCircle1(2);
                elseif pos_l>PosCircle2(2) && pos_s>PosCircle2(1)
                    pos_s_car=PosCircle1(1)+pi/2*TurningRadius+PosCircle2(2)-PosCircle1(2)+atan((pos_l-PosCircle2(2))/(eps+pos_s-PosCircle2(1)))*TurningRadius;
                else
                    pos_s_car=PosCircle1(1)+pi/2*TurningRadius+PosCircle2(2)-PosCircle1(2)+pi/2*TurningRadius+PosCircle2(1)-pos_s;
                end
                d_veh2cross_strich=pos_s_lane-pos_s_car;
                
                % 用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2
                if LaneCenterline(IndexOfLaneOppositeCar(j))<PosCircle1(2)
                    disOppositeCar2circle2=PosSOppositeCar(j)-(PosCircle1(1)+sqrt(abs(TurningRadius.^2-(LaneCenterline(IndexOfLaneOppositeCar(j))-PosCircle1(2)).^2)));
                elseif LaneCenterline(IndexOfLaneOppositeCar(j))>=PosCircle1(2)&&LaneCenterline(IndexOfLaneOppositeCar(j))<=PosCircle2(2)
                    disOppositeCar2circle2=PosSOppositeCar(j)-(PosCircle1(1)+TurningRadius);
                else
                    disOppositeCar2circle2=PosSOppositeCar(j)-(PosCircle2(1)+sqrt(abs(TurningRadius.^2-(LaneCenterline(IndexOfLaneOppositeCar(j))-PosCircle2(2)).^2)));
                end
                if IndexOfLaneOppositeCar(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(j)>0 && d_veh2cross_strich>0
                    if disOppositeCar2circle2>0
                        %                     if IndexOfLaneOppositeCar(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(j)>0 && d_veh2cross_strich>0
                        timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh)/max([SpeedOppositeCarFront(j) 0.00001])]);
                        s_max=0.5*(min([speed+a_max_com*timeGap v_max_turnAround])+speed)*timeGap;
                        if s_max<=d_veh2cross_strich+l_veh
                            AEBactive=int16(5);
                            break;
                        end
                    elseif disOppositeCar2circle2<=0
%                         if disOppositeCar2circle2>-l_veh
                        if disOppositeCar2circle2>-LengthOppositeCar(j)
                            AEBactive=int16(5);
                            break;
                        end
                    end
                end
            end
        end
        
    elseif TypeOfTurnAround==2
        if TurnAroundState==1&&PosCircle1(1)-pos_s<=0
            for j=1:length(IndexOfLaneOppositeCar)
                if IndexOfLaneOppositeCar(j)~=0
                    % 用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离)
                    if pos_s<PosCircle1(1)
                        if (LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))<TurningRadius
                            d_veh2cross_strich=PosCircle1(1)-pos_s+TurningRadius*acos((TurningRadius-(LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius)))/TurningRadius);
                        else
                            d_veh2cross_strich=PosCircle1(1)-pos_s+(TurningRadius*asin(((LaneCenterline(IndexOfLaneOppositeCar(j))-(PosCircle1(2)-TurningRadius))-TurningRadius)/TurningRadius)+TurningRadius*pi/2);
                        end
                    else
                        d_veh2cross_strich=(asin((LaneCenterline(IndexOfLaneOppositeCar(j))-PosCircle1(2))/TurningRadius)-atan2(pos_l-PosCircle1(2),pos_s-PosCircle1(1)))*TurningRadius;
                    end
                    % 用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2
                    disOppositeCar2circle2=PosSOppositeCar(j)-(PosCircle1(1)+sqrt(abs(TurningRadius.^2-(LaneCenterline(IndexOfLaneOppositeCar(j))-PosCircle1(2)).^2)));
                    if IndexOfLaneOppositeCar(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(j)>0 && d_veh2cross_strich>0
                        if disOppositeCar2circle2>0
                            
                            timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh)/max([SpeedOppositeCarFront(j) 0.00001])]);
                            s_max=0.5*(min([speed+a_max_com*timeGap v_max_turnAround])+speed)*timeGap;
                            if s_max<=d_veh2cross_strich+l_veh*double(IndexOfLaneOppositeCar(j)~=TargetLaneIndexOpposite)
                                AEBactive=int16(5);
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
%                             if disOppositeCar2circle2>-l_veh
                            if disOppositeCar2circle2>-LengthOppositeCar(j)
                                AEBactive=int16(5);
                                break;
                            end
                        end
                    end
                end
            end
        elseif TurnAroundState==2
            k=(pos_mid1_rear(1,2)-pos_mid2_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
            b=(pos_mid1_rear(1,1)*pos_mid2_rear(1,2)-pos_mid2_rear(1,1)*pos_mid1_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
            for i=1:length(IndexOfLaneOppositeCar)
                if IndexOfLaneOppositeCar(i)~=0
                    % 用pos_s、Pos_circle2、pos_l求pos_s_rear、pos_l_rear
                    pos_psi=(atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1))));
                    pos_s_rear=pos_s+sin(pos_psi)*l_veh;
                    pos_l_rear=pos_l-cos(pos_psi)*l_veh;
                    % 用IndexOfLaneOppositeCar(i)、pos_s_rear、pos_l_rear、LaneCenterline、k、b、求d_veh2cross_strich(注意是车尾到车道中心线的距离)
                    d_veh2cross_strich_s=pos_s_rear-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                    d_veh2cross_strich_l=pos_l_rear-LaneCenterline(IndexOfLaneOppositeCar(i));
                    d_veh2cross_strich=sqrt(d_veh2cross_strich_s.^2+d_veh2cross_strich_l.^2)*sign(d_veh2cross_strich_l);
                    if IndexOfLaneOppositeCar(i)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(i)>=max([1 pos_mid2_rear(4)]) && d_veh2cross_strich>0
                        disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                        if disOppositeCar2circle2>0
                            timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh)/max([SpeedOppositeCar(i) 0.00001])]);
                            s_max=0.5*(min([speed+a_max_com*timeGap v_max_turnAround])+speed)*timeGap;
                            if s_max<=d_veh2cross_strich+l_veh*double(IndexOfLaneOppositeCar(i)~=TargetLaneIndexOpposite)
                                AEBactive=int16(5);
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
%                             if disOppositeCar2circle2>-l_veh
                            if disOppositeCar2circle2>-LengthOppositeCar(i)
                                AEBactive=int16(5);
                                break;
                            end
                        end
                    end
                end
            end
            if AEBactive==0
%                 SpeedCodirectCar=SpeedCodirectCar(SpeedCodirectCar>-1);%20220324
% SpeedCodirectCar默认值为-1, LaneCenterline最后一列为掉头前所在车道
%                 for j=1:length(SpeedCodirectCar)
                 for j=1:length(SpeedCodirectCar(SpeedCodirectCar>-1))
                    % 用pos_s、Pos_circle2、pos_l求pos_s_rear、pos_l_rear
                    pos_psi=(atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1))));
                    pos_s_rear=pos_s+sin(pos_psi)*l_veh;
                    pos_l_rear=pos_l-cos(pos_psi)*l_veh;
                    % IndexOfLaneCodirectCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(默认同向第二条车道的宽度为3.2)(注意是车尾到车道中心线的距离)
                    if IndexOfLaneCodirectCar(j)==-1
                        d_veh2cross_strich_s=pos_s_rear-(LaneCenterline(length(LaneCenterline))-b)/k;
                        d_veh2cross_strich_l=pos_l_rear-LaneCenterline(length(LaneCenterline));
                        d_veh2cross_strich=sqrt(d_veh2cross_strich_s.^2+d_veh2cross_strich_l.^2)*sign(d_veh2cross_strich_l);
                    else
                        d_veh2cross_strich_s=pos_s_rear-(LaneCenterline(length(LaneCenterline))-3.2-b)/k;
                        d_veh2cross_strich_l=pos_l_rear-(LaneCenterline(length(LaneCenterline))-3.2);
                        d_veh2cross_strich=sqrt(d_veh2cross_strich_s.^2+d_veh2cross_strich_l.^2)*sign(d_veh2cross_strich_l);
                    end
                    if IndexOfLaneCodirectCar(j)>=pos_mid2_rear(4) && d_veh2cross_strich>0
                        if IndexOfLaneCodirectCar(j)==-1
                            disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-b)/k-PosSCodirectCar(j);
                        else
                            disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-3.2-b)/k-PosSCodirectCar(j);
                        end
                        if disOppositeCar2circle2>0
                            timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh)/max([SpeedCodirectCar(j) 0.00001])]);
                            s_max=0.5*(min([speed+a_max_com*timeGap v_max_turnAround])+speed)*timeGap;
                            if s_max<=d_veh2cross_strich+l_veh*double(IndexOfLaneCodirectCar(j)~=pos_mid2_rear(4))
                                AEBactive=int16(5);
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
%                             if disOppositeCar2circle2>-l_veh
                            if disOppositeCar2circle2>-LengthCodirectCar(j)
                                AEBactive=int16(5);
                                break;
                            end
                        end
                    end
                end
            end
        elseif TurnAroundState==3
            k=(pos_mid2(1,2)-pos_end(1,2))/(pos_mid2(1,1)-pos_end(1,1));
            b=(pos_mid2(1,1)*pos_end(1,2)-pos_end(1,1)*pos_mid2(1,2))/(pos_mid2(1,1)-pos_end(1,1));
            for i=1:length(IndexOfLaneOppositeCar)
                if IndexOfLaneOppositeCar(i)~=0
                    % 用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(注意是车头到车道中心线的距离)
                    d_veh2cross_strich_s=(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k-pos_s;
                    d_veh2cross_strich_l=LaneCenterline(IndexOfLaneOppositeCar(i))-pos_l;
                    d_veh2cross_strich=sqrt(d_veh2cross_strich_s.^2+d_veh2cross_strich_l.^2)*sign(d_veh2cross_strich_l);
                    
                    if IndexOfLaneOppositeCar(i)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(i)>0 && d_veh2cross_strich>0
                        disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                        if disOppositeCar2circle2>0
                            %                         timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedOppositeCar(i) 0.00001])]);
                            timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh)/max([SpeedOppositeCar(i) 0.00001])]);%20220214
                            s_max=0.5*(min([speed+a_max_com*timeGap v_max_turnAround])+speed)*timeGap;
                            if s_max<=d_veh2cross_strich+l_veh
                                AEBactive=int16(5);
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
%                             if disOppositeCar2circle2>-l_veh
                            if disOppositeCar2circle2>-LengthOppositeCar(i)
                                AEBactive=int16(5);
                                break;
                            end
                        end
                    end
                end
            end
        end
    end
end
% 输出a_soll_TrajPlanTurnAround（-4或者不变）和AEBactive=5
if AEBactive==5
    a_soll_TrajPlanTurnAround=min([-4*sign(speed),a_soll_TrajPlanTurnAround]);
end
a_sollTurnAround2Decider=a_soll_TrajPlanTurnAround;
% 一次顺车掉头轨迹生成
if TypeOfTurnAround==1
    %------------------------------------------------------------------------------------------------------------------------------------------
    if wait_turnAround==1||wait_TrafficLight==1%停车点计算
        stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
        Vend=0;
    elseif dec_trunAround==1
        stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
        Vend=v_max_turnAround;
    else
        Vend=v_max;
    end
    IsStopSpeedPlan=0;
    if Vend<speed && stopdistance<200 && (Vend.^2-speed.^2)/(-8)<=stopdistance  && AEBactive==0
        a_soll_TrajPlanTurnAround=max(a_soll_TrajPlanTurnAround,-(2/3*speed+1/3*Vend).^2/(2/3*stopdistance));
        [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
        if a_soll_TrajPlanTurnAround>=-(2/3*speed+1/3*Vend).^2/(2/3*stopdistance)
            tend=(sqrt(max(0,(2/3*speed+1/3*Vend).^2+(2/3)*a_soll_TrajPlanTurnAround*stopdistance))-2/3*speed-1/3*Vend)/(1/3*a_soll_TrajPlanTurnAround+eps);
            jerk=(Vend-speed-a_soll_TrajPlanTurnAround*tend)/(1/2*tend.^2);
            if (-a_soll_TrajPlanTurnAround/jerk)<tend&&(-a_soll_TrajPlanTurnAround/jerk)>0&&jerk>0
               tend=3*stopdistance/(speed+2*Vend);
               jerk=-(2*(Vend - speed)*(2*Vend + speed)^2)/(9*stopdistance^2);
               a_soll_limit=(2*(Vend - speed)*(2*Vend + speed))/(3*stopdistance);
                if JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_limit)==a_soll_limit
                    a_soll_TrajPlanTurnAround=a_soll_limit;
                    if a_soll_TrajPlanTurnAround<=a_soll_ACC || CurrentLaneFrontVel<0.2
                        IsStopSpeedPlan=1;
                    end
                end
            else
                if a_soll_TrajPlanTurnAround<=a_soll_ACC || CurrentLaneFrontVel<0.2
                    IsStopSpeedPlan=1;
                end
            end
            if IsStopSpeedPlan==1
                if pos_s-PosCircle1(1)>0&&pos_l<PosCircle1(2)
                    passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
                    passedPerimeter=(pi/2+passedAngle)*TurningRadius;
                elseif pos_l<=PosCircle2(2)&&pos_l>=PosCircle1(2)
                    passedPerimeter=pos_l-PosCircle1(2)+TurningRadius*pi/2;
                elseif pos_l>PosCircle2(2)
                    passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
                    if passedAngle<0
                        passedPerimeter=PosCircle2(2)-PosCircle1(2)+TurningRadius*pi+PosCircle2(1)-pos_s;
                    else
                        passedPerimeter=passedAngle*TurningRadius+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2;
                    end
                else
                    passedPerimeter=pos_s-PosCircle1(1);
                end
                for count_1=1:1:80
                    t_count_1=0.05*count_1;
                    if t_count_1<=tend
                        targetSpeed= speed+a_soll_TrajPlanTurnAround*t_count_1+0.5*jerk*t_count_1.^2;
                        adavancedPerimeter=speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
                    else
                        targetSpeed=Vend;
                        adavancedPerimeter=stopdistance+(t_count_1-tend)*Vend;
                    end
                    targetPerimeter=adavancedPerimeter+passedPerimeter;
                    if targetPerimeter<pi*TurningRadius/2
                        targetAngle=targetPerimeter/TurningRadius-pi/2;
                    elseif targetPerimeter<=pi*TurningRadius/2+PosCircle2(2)-PosCircle1(2)&&targetPerimeter>=pi*TurningRadius/2
                        targetAngle=0;
                    else
                        targetAngle=(targetPerimeter-(PosCircle2(2)-PosCircle1(2)))/TurningRadius-pi/2;
                    end
                    if targetAngle<=-pi/2
                        traj_s(count_1)=pos_s+adavancedPerimeter;
                        traj_l(count_1)=pos_l_CurrentLane;
                        traj_psi(count_1)=90;
                        traj_vs(count_1)=targetSpeed;
                        traj_vl(count_1)=0;
                        traj_omega(count_1)=0;
                    elseif targetAngle<0%PI()
                        traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                        traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
                        traj_psi(count_1)=-targetAngle*180/pi;
                        traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                        traj_vl(count_1)=targetSpeed*cos(targetAngle);
                        traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
                    elseif targetAngle==0%PI()
                        traj_s(count_1)=PosCircle1(1)+TurningRadius;
                        traj_l(count_1)=PosCircle1(2)+targetPerimeter-pi*TurningRadius/2;
                        traj_psi(count_1)=-targetAngle*180/pi;
                        traj_vs(count_1)=0;
                        traj_vl(count_1)=targetSpeed;
                        traj_omega(count_1)=targetSpeed;
                    elseif targetAngle<=pi/2%PI()
                        traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                        traj_l(count_1)=PosCircle2(2)+sin(targetAngle)*TurningRadius;
                        traj_psi(count_1)=-targetAngle*180/pi;
                        traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                        traj_vl(count_1)=targetSpeed*cos(targetAngle);
                        traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
                    else
                        traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2));
                        traj_l(count_1)=PosCircle2(2)+TurningRadius;
                        traj_psi(count_1)=-90;
                        traj_vs(count_1)=-targetSpeed;
                        traj_vl(count_1)=0;
                        traj_omega(count_1)=0;
                    end
                end
            end
        end
    end
    if IsStopSpeedPlan==0
        [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
        %------------------------------------------------------------------------------------------------------------
            if a_soll_TrajPlanTurnAround<0
                Vend=0;
                % tend=(0-speed)/(a_soll+eps);
            elseif a_soll_TrajPlanTurnAround>0
                Vend=v_max;
                % tend=(0-speed)/(a_soll+eps);
            else
                Vend=speed;
                % tend=0;
            end
        tend=(Vend-speed)/(a_soll_TrajPlanTurnAround+eps);
        %------------------------------------------------------------------------------------------------------------
        if pos_s-PosCircle1(1)>0&&pos_l<PosCircle1(2)
            passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
            passedPerimeter=(pi/2+passedAngle)*TurningRadius;
        elseif pos_l<=PosCircle2(2)&&pos_l>=PosCircle1(2)
            passedPerimeter=pos_l-PosCircle1(2)+TurningRadius*pi/2;
        elseif pos_l>PosCircle2(2)
            passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
            if passedAngle<0
                passedPerimeter=PosCircle2(2)-PosCircle1(2)+TurningRadius*pi+PosCircle2(1)-pos_s;
            else
                passedPerimeter=passedAngle*TurningRadius+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2;
            end
        else
            passedPerimeter=pos_s-PosCircle1(1);
        end
        for count_1=1:1:80
            t_count_1=0.05*count_1;
            if t_count_1<=tend
                targetSpeed= speed+a_soll_TrajPlanTurnAround*t_count_1;
                adavancedPerimeter=speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2;
            else
                targetSpeed=Vend;
                adavancedPerimeter=(Vend.^2-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps)+(t_count_1-tend)*Vend;
            end
            targetPerimeter=adavancedPerimeter+passedPerimeter;
            if targetPerimeter<pi*TurningRadius/2
                targetAngle=targetPerimeter/TurningRadius-pi/2;
            elseif targetPerimeter<=pi*TurningRadius/2+PosCircle2(2)-PosCircle1(2)&&targetPerimeter>=pi*TurningRadius/2
                targetAngle=0;
            else
                targetAngle=(targetPerimeter-(PosCircle2(2)-PosCircle1(2)))/TurningRadius-pi/2;
            end
            if targetAngle<=-pi/2
                traj_s(count_1)=pos_s+adavancedPerimeter;
                traj_l(count_1)=pos_l_CurrentLane;
                traj_psi(count_1)=90;
                traj_vs(count_1)=targetSpeed;
                traj_vl(count_1)=0;
                traj_omega(count_1)=0;
            elseif targetAngle<0%PI()
                %         targetAngle=targetAngle-pi/2;
                traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
                traj_psi(count_1)=-targetAngle*180/pi;
                traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                traj_vl(count_1)=targetSpeed*cos(targetAngle);
                traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
            elseif targetAngle==0%PI()
                traj_s(count_1)=PosCircle1(1)+TurningRadius;
                traj_l(count_1)=PosCircle1(2)+targetPerimeter-pi*TurningRadius/2;
                traj_psi(count_1)=-targetAngle*180/pi;
                traj_vs(count_1)=0;
                traj_vl(count_1)=targetSpeed;
                traj_omega(count_1)=targetSpeed;
            elseif targetAngle<=pi/2%PI()
                traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                traj_l(count_1)=PosCircle2(2)+sin(targetAngle)*TurningRadius;
                traj_psi(count_1)=-targetAngle*180/pi;
                traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                traj_vl(count_1)=targetSpeed*cos(targetAngle);
                traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
            else
                traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2));
                traj_l(count_1)=PosCircle2(2)+TurningRadius;
                traj_psi(count_1)=-90;
                traj_vs(count_1)=-targetSpeed;
                traj_vl(count_1)=0;
                traj_omega(count_1)=0;
            end
        end
    end
%------------------------------------------------------------------------------------------------------------------------------------------
%     if wait_turnAround==1||wait_TrafficLight==1%停车点计算
%         stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
%     end
%     IsStopSpeedPlan=0;
%     if stopdistance<200&&speed.^2/8<=stopdistance && (-((4/9)*speed.^2/(2/3*stopdistance))<=a_soll_ACC || CurrentLaneFrontVel<0.2)&&AEBactive==0
%         a_soll_TrajPlanTurnAround=max(a_soll_TrajPlanTurnAround,-((4/9)*speed.^2/(2/3*stopdistance)));
%         [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
%         if a_soll_TrajPlanTurnAround>=-((4/9)*speed.^2/(2/3*stopdistance))
%             tend=(3*sqrt(max(0,(4/9)*speed.^2+(2/3)*a_soll_TrajPlanTurnAround*stopdistance))-2*speed)/(a_soll_TrajPlanTurnAround+eps);
%             jerk=-2*(speed+a_soll_TrajPlanTurnAround*tend)/(tend.^2);
%             for count_1=1:1:80
%                 t_count_1=0.05*count_1;
%                 if t_count_1<=tend
%                     traj_vs(count_1)= speed+a_soll_TrajPlanTurnAround*t_count_1+0.5*jerk*t_count_1.^2;
%                     traj_s(count_1)=pos_s+speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
%                 else
%                     traj_vs(count_1)=0;
%                     traj_s(count_1)=pos_s+stopdistance;
%                 end
%                 traj_vl(count_1)=0;
%                 traj_omega(count_1)=0;
%                 traj_l(count_1)=pos_l_CurrentLane;
%                 traj_psi(count_1)=90;
%                 IsStopSpeedPlan=1;
%             end
%         end
%     end
%     if IsStopSpeedPlan==0
%         [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
%         if pos_s-PosCircle1(1)>0&&pos_l<PosCircle1(2)
%             passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
%             passedPerimeter=(pi/2+passedAngle)*TurningRadius;
%         elseif pos_l<=PosCircle2(2)&&pos_l>=PosCircle1(2)
%             passedPerimeter=pos_l-PosCircle1(2)+TurningRadius*pi/2;
%         elseif pos_l>PosCircle2(2)
%             passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
%             if passedAngle<0
%                 passedPerimeter=PosCircle2(2)-PosCircle1(2)+TurningRadius*pi+PosCircle2(1)-pos_s;
%             else
%                 passedPerimeter=passedAngle*TurningRadius+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2;
%             end
%         else
%             passedPerimeter=pos_s-PosCircle1(1);
%         end
%         for count_1=1:1:80
%             t_count_1=0.05*count_1;
%             targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%             if targetSpeed==0
%                 adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
%             else
%                 adavancedPerimeter=(targetSpeed+speed)*t_count_1/2;
%             end
%             targetPerimeter=adavancedPerimeter+passedPerimeter;
%             if targetPerimeter<pi*TurningRadius/2
%                 targetAngle=targetPerimeter/TurningRadius-pi/2;
%             elseif targetPerimeter<=pi*TurningRadius/2+PosCircle2(2)-PosCircle1(2)&&targetPerimeter>=pi*TurningRadius/2
%                 targetAngle=0;
%             else
%                 targetAngle=(targetPerimeter-(PosCircle2(2)-PosCircle1(2)))/TurningRadius-pi/2;
%             end
%             if targetAngle<=-pi/2
%                 traj_s(count_1)=pos_s+adavancedPerimeter;
%                 traj_l(count_1)=pos_l_CurrentLane;
%                 traj_psi(count_1)=90;
%                 traj_vs(count_1)=targetSpeed;
%                 traj_vl(count_1)=0;
%                 traj_omega(count_1)=0;
%             elseif targetAngle<0%PI()
%                 %         targetAngle=targetAngle-pi/2;
%                 traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
%                 traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
%                 traj_psi(count_1)=-targetAngle*180/pi;
%                 traj_vs(count_1)=-targetSpeed*sin(targetAngle);
%                 traj_vl(count_1)=targetSpeed*cos(targetAngle);
%                 traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
%             elseif targetAngle==0%PI()
%                 traj_s(count_1)=PosCircle1(1)+TurningRadius;
%                 traj_l(count_1)=PosCircle1(2)+targetPerimeter-pi*TurningRadius/2;
%                 traj_psi(count_1)=-targetAngle*180/pi;
%                 traj_vs(count_1)=0;
%                 traj_vl(count_1)=targetSpeed;
%                 traj_omega(count_1)=targetSpeed;
%             elseif targetAngle<=pi/2%PI()
%                 traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
%                 traj_l(count_1)=PosCircle2(2)+sin(targetAngle)*TurningRadius;
%                 traj_psi(count_1)=-targetAngle*180/pi;
%                 traj_vs(count_1)=-targetSpeed*sin(targetAngle);
%                 traj_vl(count_1)=targetSpeed*cos(targetAngle);
%                 traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
%             else
%                 traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2));
%                 traj_l(count_1)=PosCircle2(2)+TurningRadius;
%                 traj_psi(count_1)=-90;
%                 traj_vs(count_1)=-targetSpeed;
%                 traj_vl(count_1)=0;
%                 traj_omega(count_1)=0;
%             end
%         end
%     end
%------------------------------------------------------------------------------------------------------------------------------------------
    %一次掉头结束判断
    if traj_psi(1)==-90 && pos_s<PosCircle1(1)-TurningRadius/2
        TurnAroundActive=int16(0);
    end
    a_sollTurnAround2Decider=a_soll_TrajPlanTurnAround;
end
% 二次顺车掉头轨迹生成
passedPerimeter=0;%20220324
if TypeOfTurnAround==2
    %---------------------------------------------------------------------------------------------------------------------------------------------
    if TurnAroundState==1||TurnAroundState==0
        if pos_start(1)-pos_s>0
            dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_start(2)-PosCircle1(2)),(pos_start(1)-PosCircle1(1))))*TurningRadius+pos_start(1)-pos_s;
        else
            dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_l-PosCircle1(2)),(pos_s-PosCircle1(1))))*TurningRadius;
        end
        if TurnAroundState==0
            if wait_turnAround==1||wait_TrafficLight==1%停车点计算
                stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
                Vend=0;
            elseif dec_trunAround==1%减速
                stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
                Vend=v_max_turnAround;
            else
                stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance);
                Vend=v_max;
            end
        else % TurnAroundState==1%停车点计算
            stopdistance=min(dis2pos_mid1,stopdistance);
            Vend=0;
        end
        IsStopSpeedPlan=0;
        if Vend<speed && stopdistance<200 && (Vend.^2-speed.^2)/(-8)<=stopdistance  && AEBactive==0
            a_soll_TrajPlanTurnAround=max(a_soll_TrajPlanTurnAround,-(2/3*speed+1/3*Vend).^2/(2/3*stopdistance));
            [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
            if a_soll_TrajPlanTurnAround>=-(2/3*speed+1/3*Vend).^2/(2/3*stopdistance)
                tend=(sqrt(max(0,(2/3*speed+1/3*Vend).^2+(2/3)*a_soll_TrajPlanTurnAround*stopdistance))-2/3*speed-1/3*Vend)/(1/3*a_soll_TrajPlanTurnAround+eps);
                jerk=(Vend-speed-a_soll_TrajPlanTurnAround*tend)/(1/2*tend.^2);
                if (-a_soll_TrajPlanTurnAround/jerk)<tend&&(-a_soll_TrajPlanTurnAround/jerk)>0&&jerk>0
                    tend=3*stopdistance/(speed+2*Vend);
                    jerk=-(2*(Vend - speed)*(2*Vend + speed)^2)/(9*stopdistance^2);
                    a_soll_limit=(2*(Vend - speed)*(2*Vend + speed))/(3*stopdistance);
                    if JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_limit)==a_soll_limit
                        a_soll_TrajPlanTurnAround=a_soll_limit;
                        if a_soll_TrajPlanTurnAround<=a_soll_ACC || CurrentLaneFrontVel<0.2
                            IsStopSpeedPlan=1;
                        end
                    end
                else
                    if a_soll_TrajPlanTurnAround<=a_soll_ACC || CurrentLaneFrontVel<0.2
                        IsStopSpeedPlan=1;
                    end
                end
                if IsStopSpeedPlan==1
                    if pos_s-PosCircle1(1)>0
                        passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
                        passedPerimeter=(pi/2+passedAngle)*TurningRadius;
                    elseif pos_s-PosCircle1(1)<0&&pos_l>PosCircle1(2)
                        passedPerimeter=PosCircle1(1)-pos_s+TurningRadius*pi;
                    else
                        passedPerimeter=pos_s-PosCircle1(1);
                    end
                    for count_1=1:1:80
                        t_count_1=0.05*count_1;
                        if t_count_1<=tend
                            targetSpeed= speed+a_soll_TrajPlanTurnAround*t_count_1+0.5*jerk*t_count_1.^2;
                            adavancedPerimeter=speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
                        else
                            targetSpeed=Vend;
                            adavancedPerimeter=stopdistance+(t_count_1-tend)*Vend;
                        end
                        targetPerimeter=adavancedPerimeter+passedPerimeter;
                        targetAngle=targetPerimeter/TurningRadius-pi/2;
                        if targetAngle<=-pi/2
                            traj_s(count_1)=pos_s+adavancedPerimeter;
                            traj_l(count_1)=pos_l;
                            traj_psi(count_1)=90;
                            traj_vs(count_1)=targetSpeed;
                            traj_vl(count_1)=0;
                            traj_omega(count_1)=0;
                        elseif targetAngle<pi/2
                            traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                            traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
                            traj_psi(count_1)=-targetAngle*180/pi;
                            traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                            traj_vl(count_1)=targetSpeed*cos(targetAngle);
                            traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
                        else
                            traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi);
                            traj_l(count_1)=PosCircle1(2)+TurningRadius;
                            traj_psi(count_1)=-90;
                            traj_vs(count_1)=-targetSpeed;
                            traj_vl(count_1)=0;
                            traj_omega(count_1)=0;
                        end
                    end
                end
            end
        end
        if IsStopSpeedPlan==0
            [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
            %------------------------------------------------------------------------------------------------------------
            if a_soll_TrajPlanTurnAround<0
                Vend=0;
            elseif a_soll_TrajPlanTurnAround>0
                Vend=v_max;
            else
                Vend=speed;
            end
            tend=(Vend-speed)/(a_soll_TrajPlanTurnAround+eps);
           %------------------------------------------------------------------------------------------------------------
            if pos_s-PosCircle1(1)>0
                passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1)));
                passedPerimeter=(pi/2+passedAngle)*TurningRadius;
            elseif pos_s-PosCircle1(1)<0&&pos_l>PosCircle1(2)
                passedPerimeter=PosCircle1(1)-pos_s+TurningRadius*pi;
            else
                passedPerimeter=pos_s-PosCircle1(1);
            end
            for count_1=1:1:80
                t_count_1=0.05*count_1;
                if t_count_1<=tend
                    targetSpeed= speed+a_soll_TrajPlanTurnAround*t_count_1;
                    adavancedPerimeter=speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2;
                else
                    targetSpeed=Vend;
                    adavancedPerimeter=(Vend.^2-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps)+(t_count_1-tend)*Vend;
                end
                targetPerimeter=adavancedPerimeter+passedPerimeter;
                targetAngle=targetPerimeter/TurningRadius-pi/2;
                if targetAngle<=-pi/2
                    traj_s(count_1)=pos_s+adavancedPerimeter;
                    traj_l(count_1)=pos_l;
                    traj_psi(count_1)=90;
                    traj_vs(count_1)=targetSpeed;
                    traj_vl(count_1)=0;
                    traj_omega(count_1)=0;
                elseif targetAngle<pi/2
                    traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
                    traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
                    traj_psi(count_1)=-targetAngle*180/pi;
                    traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                    traj_vl(count_1)=targetSpeed*cos(targetAngle);
                    traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
                else
                    traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi);
                    traj_l(count_1)=PosCircle1(2)+TurningRadius;
                    traj_psi(count_1)=-90;
                    traj_vs(count_1)=-targetSpeed;
                    traj_vl(count_1)=0;
                    traj_omega(count_1)=0;
                end
            end
        end
    elseif TurnAroundState==2
        %停车点计算  
        stopdistance=min(dis2pos_mid2,stopdistance);
        IsStopSpeedPlan=0;
        if stopdistance<200&&speed.^2/8<=stopdistance && (-((4/9)*speed.^2/(2/3*stopdistance))<=a_soll_ACC || CurrentLaneFrontVel<0.2)&&AEBactive==0
            a_soll_TrajPlanTurnAround=max(a_soll_TrajPlanTurnAround,-((4/9)*speed.^2/(2/3*stopdistance)));
            [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
            if a_soll_TrajPlanTurnAround>=-((4/9)*speed.^2/(2/3*stopdistance))
                tend=(3*sqrt(max(0,(4/9)*speed.^2+(2/3)*a_soll_TrajPlanTurnAround*stopdistance))-2*speed)/(a_soll_TrajPlanTurnAround+eps);
                jerk=-2*(speed+a_soll_TrajPlanTurnAround*tend)/(tend.^2);
                if pos_s-PosCircle2(1)<0
                    passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
                    passedPerimeter=(pi/2+passedAngle)*TurningRadius;
                else
                    passedPerimeter=pos_s-PosCircle2(1)+TurningRadius*pi;
                end
                for count_1=1:1:80
                    t_count_1=0.05*count_1;
                    if t_count_1<=tend
                        targetSpeed=speed+a_soll_TrajPlanTurnAround*t_count_1+0.5*jerk*t_count_1.^2;
                        adavancedPerimeter=speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2+(1/6)*jerk*t_count_1.^3;
                    else
                        targetSpeed=0;
                        adavancedPerimeter=stopdistance;  
                    end
                    targetPerimeter=adavancedPerimeter+passedPerimeter;
                    targetAngle=targetPerimeter/TurningRadius-pi/2;
                    if targetAngle<pi/2
                        traj_s(count_1)=PosCircle2(1)-cos(targetAngle)*TurningRadius;
                        traj_l(count_1)=PosCircle2(2)-sin(targetAngle)*TurningRadius;
                        traj_psi(count_1)=-targetAngle*180/pi;
                        traj_vs(count_1)=targetSpeed*sin(targetAngle);
                        traj_vl(count_1)=-targetSpeed*cos(targetAngle);
                        traj_omega(count_1)=targetSpeed/TurningRadius*180/pi;
                    else
                        traj_s(count_1)=PosCircle2(1)+(targetPerimeter-TurningRadius*pi);
                        traj_l(count_1)=PosCircle2(2)-TurningRadius;
                        traj_psi(count_1)=-90;
                        traj_vs(count_1)=targetSpeed;
                        traj_vl(count_1)=0;
                        traj_omega(count_1)=0;
                    end
                end
                IsStopSpeedPlan=1;
            end
        end
        if IsStopSpeedPlan==0
            [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround);
            if pos_s-PosCircle2(1)<0
                passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1)));
                passedPerimeter=(pi/2+passedAngle)*TurningRadius;
            else
                passedPerimeter=pos_s-PosCircle2(1)+TurningRadius*pi;
            end
            for count_1=1:1:80
                t_count_1=0.05*count_1;
                targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
                if targetSpeed==0
                    adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
                else
                    adavancedPerimeter=(targetSpeed+speed)*t_count_1/2;
                end
                targetPerimeter=adavancedPerimeter+passedPerimeter;
                targetAngle=targetPerimeter/TurningRadius-pi/2;
                if targetAngle<pi/2
                    traj_s(count_1)=PosCircle2(1)-cos(targetAngle)*TurningRadius;
                    traj_l(count_1)=PosCircle2(2)-sin(targetAngle)*TurningRadius;
                    traj_psi(count_1)=-targetAngle*180/pi;
                    traj_vs(count_1)=targetSpeed*sin(targetAngle);
                    traj_vl(count_1)=-targetSpeed*cos(targetAngle);
                    traj_omega(count_1)=targetSpeed/TurningRadius*180/pi;
                else
                    traj_s(count_1)=PosCircle2(1)+(targetPerimeter-TurningRadius*pi);
                    traj_l(count_1)=PosCircle2(2)-TurningRadius;
                    traj_psi(count_1)=-90;
                    traj_vs(count_1)=targetSpeed;
                    traj_vl(count_1)=0;
                    traj_omega(count_1)=0;
                end
            end
        end
     elseif TurnAroundState==3
        if pos_s-PosCircle3(1)>0
            passedAngle=atan((pos_l-PosCircle3(2))/(pos_s-PosCircle3(1)));
            passedPerimeter=(pi/2+passedAngle)*TurningRadius;
        elseif pos_s-PosCircle3(1)<0&&pos_l>PosCircle3(2)
            passedPerimeter=PosCircle3(1)-pos_s+TurningRadius*pi;
        end
        for count_1=1:1:80
            t_count_1=0.05*count_1;
            targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
            if targetSpeed==0
                adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
            else
                adavancedPerimeter=(targetSpeed+speed)*t_count_1/2;
            end
            targetPerimeter=adavancedPerimeter+passedPerimeter;
            targetAngle=targetPerimeter/TurningRadius-pi/2;
            if targetAngle<pi/2
                traj_s(count_1)=PosCircle3(1)+cos(targetAngle)*TurningRadius;
                traj_l(count_1)=PosCircle3(2)+sin(targetAngle)*TurningRadius;
                traj_psi(count_1)=-targetAngle*180/pi;
                traj_vs(count_1)=-targetSpeed*sin(targetAngle);
                traj_vl(count_1)=targetSpeed*cos(targetAngle);
                traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
            else
                traj_s(count_1)=PosCircle3(1)-(targetPerimeter-TurningRadius*pi);
                traj_l(count_1)=PosCircle3(2)+TurningRadius;
                traj_psi(count_1)=-90;
                traj_vs(count_1)=-targetSpeed;
                traj_vl(count_1)=0;
                traj_omega(count_1)=0;
            end
        end
    end    
    %---------------------------------------------------------------------------------------------------------------------------------------------
    a_sollTurnAround2Decider=a_soll_TrajPlanTurnAround;
end
GlobVars.Decider.a_sollpre2traj=a_soll_TrajPlanTurnAround;
GlobVars.TrajPlanTurnAround.posCircle=PosCircle1;
GlobVars.TrajPlanTurnAround.posCircle2=PosCircle2;
GlobVars.TrajPlanTurnAround.posCircle3=PosCircle3;
GlobVars.TrajPlanTurnAround.pos_start=pos_start;
GlobVars.TrajPlanTurnAround.pos_mid1=pos_mid1;
GlobVars.TrajPlanTurnAround.pos_mid2=pos_mid2;
GlobVars.TrajPlanTurnAround.pos_mid1_rear=pos_mid1_rear;
GlobVars.TrajPlanTurnAround.pos_mid2_rear=pos_mid2_rear;
GlobVars.TrajPlanTurnAround.pos_end=pos_end;
GlobVars.TrajPlanTurnAround.laneCenterline=LaneCenterline;
GlobVars.TrajPlanTurnAround.dec_trunAround=dec_trunAround;
GlobVars.TrajPlanTurnAround.wait_turnAround=wait_turnAround;
GlobVars.TrajPlanTurnAround.typeOfTurnAround=TypeOfTurnAround;
GlobVars.TrajPlanTurnAround.turnAroundState=TurnAroundState;
GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite=TargetLaneIndexOpposite;
end