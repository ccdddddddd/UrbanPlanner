function [a_soll_TrajPlanTurnAround,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,...,
    TargetLaneIndexOpposite,TargetGear,TurnAroundState,TypeOfTurnAround,TurnAroundActive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,TurningRadius,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLaneCurrent,s_turnaround_border,...,
    PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_mid1_rear,pos_mid2_rear,pos_end,LaneCenterline,dec_trunAround,wait_turnAround,IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,IndexOfLaneCodirectCar,SpeedCodirectCar,PosSCodirectCar,...,
    CurrentLane,v_max,a_soll,wait_TrafficLight,CurrentGear,TypeOfTurnAround,TurnAroundState,TargetLaneIndexOpposite,TurnAroundActive)

w_veh=1.8;
l_veh=5;
D_safe=0.5;
dec2line=0.2;
a_min=-2;
a_max_com=1.5;
v_max_turnAround=5;
d_veh2cross=zeros([5,1]);
traj_s=zeros([1 80]);
traj_l=zeros([1 80]);
traj_psi=zeros([1 80]);
traj_vs=zeros([1 80]);
traj_vl=zeros([1 80]);
traj_omega=zeros([1 80]);
TargetGear=CurrentGear;
IndexOfLaneOppositeCarFront=zeros([6,1]);
SpeedOppositeCarFront=zeros([6,1]);
PosSOppositeCarFront=zeros([6,1]);
PosSOppositeCarFront=PosSOppositeCarFront+200;
SpeedOppositeCarRear=zeros([6,1]);
PosSOppositeCarRear=zeros([6,1]);
PosSOppositeCarRear=PosSOppositeCarRear-200;
% 目标车道选择
if TypeOfTurnAround==0
    % d_cur2tar=0.5*(WidthOfLanesOpposite(1)-w_veh)+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(1);
    TargetLaneIndexOpposite=1;
    TypeOfTurnAround=2; % 1为一次顺车掉头，2为二次顺车掉头
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
            TypeOfTurnAround=1;
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        elseif 0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar>=2*TurningRadius
            % d_cur2tar=0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar;
            d_cur2tar=2*TurningRadius;
            TargetLaneIndexOpposite=i;
            TypeOfTurnAround=1;
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        end
    end
    % 一次顺车掉头路径生成
    if TypeOfTurnAround==1
        PosCircle1(1)=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe;
        PosCircle1(2)=pos_l_CurrentLane+TurningRadius;
        % s_start=PosCircle1(1);
        PosCircle2(1)=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe;
        PosCircle2(2)=pos_l_TargetLane-TurningRadius;
        LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite); % 车道中心线位置 全局变量
    end
    % 二次顺车掉头路径生成
    if TypeOfTurnAround==2
        pos_l_TargetLane=pos_l_CurrentLane+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(NumOfLanesOpposite)+...,
            sum(WidthOfLanesOpposite(1:NumOfLanesOpposite-1))+0.5*(WidthOfLanesOpposite(NumOfLanesOpposite)-w_veh)-dec2line;
        TargetLaneIndexOpposite=NumOfLanesOpposite;
        pos_l_TargetLaneBoundary=pos_l_CurrentLane+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(NumOfLanesOpposite)+sum(WidthOfLanesOpposite(1:NumOfLanesOpposite-1));
        [PosCircle1,PosCircle2,PosCircle3,pos_start,pos_mid1,pos_mid2,pos_end]=PathPlanTurnAround(s_turnaround_border,w_veh,TurningRadius,D_safe,pos_l_CurrentLane,pos_l_TargetLane,pos_l_TargetLaneBoundary,dec2line);
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
if TypeOfTurnAround==1
    posOfLaneCenterline=zeros([6,1]);
    for i=1:length(LaneCenterline)
        if LaneCenterline(i)~=0
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
        if LaneCenterline(i)~=0
            posOfLaneCenterline(i)=sqrt(TurningRadius^2-(PosCircle1(2)-LaneCenterline(i))^2)+PosCircle1(1);
        end
    end
end
if TypeOfTurnAround==1 ||(TypeOfTurnAround==2 && TurnAroundState<2)
    j=1;
    for i=1:length(IndexOfLaneOppositeCar)
        if IndexOfLaneOppositeCar(i)~=0
            if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i)) >= 0
                IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                PosSOppositeCarFront(j)=PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                SpeedOppositeCarFront(j)=SpeedOppositeCar(i);
                j=j+1;
            end
        end
    end
    j=1;
    for  i=1:length(IndexOfLaneOppositeCar)
        if IndexOfLaneOppositeCar(i)~=0
            if i==1
                if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i)) < 0
                    PosSOppositeCarRear(j)=posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                    SpeedOppositeCarRear(j)=SpeedOppositeCar(i);
                end
            else
                if IndexOfLaneOppositeCar(i-1)==IndexOfLaneOppositeCar(i)
                    if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i))<0&&PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i))>PosSOppositeCarRear(j)
                        PosSOppositeCarRear(j)=PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                        SpeedOppositeCarRear(j)=SpeedOppositeCar(i);
                    end
                else
                    j=j+1;
                    if PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i)) < 0
                        PosSOppositeCarRear(j)=PosSOppositeCar(i)-posOfLaneCenterline(IndexOfLaneOppositeCar(i));
                        SpeedOppositeCarRear(j)=SpeedOppositeCar(i);
                    end
                end
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
            dec_trunAround=1;
        end
    else
        if PosCircle1(1)-pos_s<=0 || wait_turnAround==1 || pos_l>PosCircle1(2)%20220225
            dec_trunAround=0;
        end
    end
    % 停车决策
    if dec_trunAround==1
        %         for i=1:TargetLaneIndexOpposite
        %             d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确
        %         end
        for j=1:length(IndexOfLaneOppositeCarFront)
            if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                d_veh=max([(d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh)/max([speed 0.00001])*SpeedOppositeCarFront(j)+0.5*w_veh+l_veh 0]);
                if PosSOppositeCarFront(j)<=d_veh
                    wait_turnAround=1;
                    break;
                end
            end
        end
        if wait_turnAround==0
            for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
                if PosSOppositeCarRear(j)>-l_veh
                    wait_turnAround=1;
                    break;
                end
            end
        end
    end
    % 起步决策
    if wait_turnAround==1 && PosCircle1(1)-pos_s<10
        wait_turnAround=0;
        a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
        for j=1:length(IndexOfLaneOppositeCarFront)
            if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                %                 timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh-TurningRadius)/max([SpeedOppositeCarFront(j) 0.00001])]);
                a_OppositeCarFront=max([ACC(50/3.6,SpeedOppositeCarRear(IndexOfLaneOppositeCarFront(j)),PosSOppositeCarFront(j)-PosSOppositeCarRear(IndexOfLaneOppositeCarFront(j)),SpeedOppositeCarFront(j),0) 0]);
                if SpeedOppositeCarFront(j)<=0.01
                    a_OppositeCarFront=0;
                end
                [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)])),...,
                    [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]);
                s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                if s_max<=d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh
                    wait_turnAround=1;
                    break;
                end
            end
        end
        if wait_turnAround==0
            for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
                if PosSOppositeCarRear(j)>-l_veh
                    wait_turnAround=1;
                    break;
                end
            end
        end
        if wait_turnAround==0
            wait_turnAround
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
    if TurnAroundState==0&&pos_s>=s_start-1&&pos_l<l_start+0.5&&pos_l>l_start-0.5
        TurnAroundState=1;
        TargetGear=4;
    end
    if TurnAroundState==1
        r1=sqrt((s_p1-pos_s)^2+(l_p1-pos_l)^2);
        r1
        if r1<0.15
            r1
        end
        if r1<0.15&&speed<=0.05
            TargetGear=2;
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
                TurnAroundState=2;
                a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
                for i=1:length(IndexOfLaneOppositeCar)
                    if ismember(IndexOfLaneOppositeCar(i),Lanes2Search)
                        k=(pos_mid1_rear(1,2)-pos_mid2_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                        b=(pos_mid1_rear(1,1)*pos_mid2_rear(1,2)-pos_mid2_rear(1,1)*pos_mid1_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                        disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                        if disOppositeCar2circle2>0
                            timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedOppositeCar(i) 0.00001])]);
                            s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                            d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(IndexOfLaneOppositeCar(i))).^2);
                            if s_max<=d_veh2cross_strich+l_veh
                                TurnAroundState=1;
                                break;
                            end
                        elseif disOppositeCar2circle2<=0
                            if disOppositeCar2circle2>-l_veh
                                TurnAroundState=1;
                                break;
                            end
                        end
                    end
                end
                if TurnAroundState==2
                    SpeedCodirectCar=SpeedCodirectCar(SpeedCodirectCar>-1); % SpeedCodirectCar默认值为-1, LaneCenterline最后一列为掉头前所在车道
                    for j=1:length(SpeedCodirectCar)
                        if ismember(IndexOfLaneCodirectCar(j),Lanes2Search)
                            k=(pos_mid1_rear(1,2)-pos_mid2_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                            b=(pos_mid1_rear(1,1)*pos_mid2_rear(1,2)-pos_mid2_rear(1,1)*pos_mid1_rear(1,2))/(pos_mid1_rear(1,1)-pos_mid2_rear(1,1));
                            if IndexOfLaneCodirectCar(j)==-1
                                disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-b)/k-PosSCodirectCar(j);
                            else
                                disOppositeCar2circle2=(LaneCenterline(length(LaneCenterline))-3.2-b)/k-PosSCodirectCar(j);
                            end
                            if disOppositeCar2circle2>0
                                timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedCodirectCar(j) 0.00001])]);
                                s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                                if IndexOfLaneCodirectCar(j)==-1
                                    d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(length(LaneCenterline))-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(length(LaneCenterline))).^2);
                                else
                                    d_veh2cross_strich=sqrt((pos_mid1_rear(1)-(LaneCenterline(length(LaneCenterline))-3.2-b)/k).^2+(pos_mid1_rear(2)-LaneCenterline(length(LaneCenterline))+3.2).^2);
                                end
                                if s_max<=d_veh2cross_strich+l_veh
                                    TurnAroundState=1;
                                    break;
                                end
                            elseif disOppositeCar2circle2<=0
                                if disOppositeCar2circle2>-l_veh
                                    TurnAroundState=1;
                                    break;
                                end
                            end
                        end
                    end
                end
            else
                TurnAroundState=1;
            end
        end
    elseif TurnAroundState==2
        r2=sqrt((s_p2-pos_s)^2+(l_p2-pos_l)^2);
        if r2<0.15&&speed<=0.05
            TargetGear=4;
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
            TurnAroundState=3;
            a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
            a_predict
            for i=1:length(IndexOfLaneOppositeCar)
                if ismember(IndexOfLaneOppositeCar(i),Lanes2Search)
                    k=(pos_mid2(1,2)-pos_end(1,2))/(pos_mid2(1,1)-pos_end(1,1));
                    b=(pos_mid2(1,1)*pos_end(1,2)-pos_end(1,1)*pos_mid2(1,2))/(pos_mid2(1,1)-pos_end(1,1));
                    disOppositeCar2circle2=PosSOppositeCar(i)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k;
                    if disOppositeCar2circle2>0
                        % a_OppositeCarFront=min(a_OppositeCarFront,1.5);
                        % [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])),...,
                        %     [0-0.01 0.01+max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]);
                        timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedOppositeCar(i) 0.00001])]);
                        s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                        d_veh2cross_strich=sqrt((pos_mid2(1)-(LaneCenterline(IndexOfLaneOppositeCar(i))-b)/k).^2+(pos_mid2(2)-LaneCenterline(IndexOfLaneOppositeCar(i))).^2);
                        if s_max<=d_veh2cross_strich+l_veh
                            TurnAroundState=2;
                            break;
                        end
                    elseif disOppositeCar2circle2<=0
                        if disOppositeCar2circle2>-l_veh
                            TurnAroundState=2;
                            break;
                        end
                    end
                end
            end
        else
            TurnAroundState=2;
        end
    elseif TurnAroundState==3
        TargetGear=4;
        if pos_s<s_end-4&&pos_l<l_end+0.5&&pos_l>l_end-0.5
            % TurnAroundState=0;
            TurnAroundActive=0;
        end
    else
        TargetGear=4;
    end
    % 进入掉头路径前决策
    for i=1:TargetLaneIndexOpposite
        if 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))<TurningRadius
            d_veh2cross(i)=PosCircle1(1)-pos_s+TurningRadius*acos((TurningRadius-(0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))))/TurningRadius);
        else
            d_veh2cross(i)=PosCircle1(1)-pos_s+(TurningRadius*asin((0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+...,
                sum(WidthOfLanesOpposite(1:i-1))-TurningRadius)/TurningRadius)+TurningRadius*pi/2);
        end
    end
    if TurnAroundState==0 || (TurnAroundState==1 && pos_start(1)-pos_s>=0)
        % 策略模式判断
        d_bre=(0-speed.^2)/(2*a_min);
        if dec_trunAround==0
            if pos_start(1)-pos_s<=d_bre+2*l_veh && pos_start(1)-pos_s>0 && pos_l<PosCircle1(2) && TurnAroundState<2
                dec_trunAround=1;
            end
        else
            if (pos_start(1)-pos_s<=0 || wait_turnAround==1) && pos_l<PosCircle1(2) && TurnAroundState<2
                dec_trunAround=0;
            end
        end
        
        % 停车决策
        if dec_trunAround==1
            for j=1:length(IndexOfLaneOppositeCarFront)
                if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                    d_veh=max([(d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh)/max([speed 0.00001])*SpeedOppositeCarFront(j)+0.5*w_veh+l_veh 0]);
                    if PosSOppositeCarFront(j)<=d_veh
                        wait_turnAround=1;
                        break;
                    end
                end
            end
            if wait_turnAround==0
                for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
                    if PosSOppositeCarRear(j)>-l_veh
                        wait_turnAround=1;
                        break;
                    end
                end
            end
        end
        % 起步决策
        if wait_turnAround==1 && pos_start(1)-pos_s<10
            wait_turnAround=0;
            % a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
            dis2pos_mid1=abs(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_l-PosCircle1(2)),(pos_s-PosCircle1(1))))*TurningRadius;
            a_predict=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround) ACC(v_max_turnAround,0,dis2pos_mid1+9,speed,wait_turnAround)]);
            for j=1:length(IndexOfLaneOppositeCarFront)
                if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                    a_OppositeCarFront=max([ACC(50/3.6,SpeedOppositeCarRear(IndexOfLaneOppositeCarFront(j)),PosSOppositeCarFront(j)-PosSOppositeCarRear(IndexOfLaneOppositeCarFront(j)),SpeedOppositeCarFront(j),0) 0]);
                    if SpeedOppositeCarFront(j)<=0.01
                         a_OppositeCarFront=0;
                    end
                    a_OppositeCarFront=min(a_OppositeCarFront,1.5);
                    [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)])),...,
                        [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]);
                    % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)/max([SpeedOppositeCarFront(j) 0.00001])]);
                    s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap;
                    if s_max<=d_veh2cross(IndexOfLaneOppositeCarFront(j))+l_veh
                        wait_turnAround=1;
                        break;
                    end
                end
            end
            if wait_turnAround==0
                for j=1:min([length(PosSOppositeCarRear) TargetLaneIndexOpposite])
                    if PosSOppositeCarRear(j)>-l_veh
                        wait_turnAround=1;
                        break;
                    end
                end
            end
            if wait_turnAround==0
            end
        end
    end
end

% ACC速度规划
if TypeOfTurnAround==1
    % 一次顺车掉头速度规划
    if (wait_turnAround==1 && PosCircle1(1)>pos_s) || wait_TrafficLight==1
        % a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]);
        a_soll_TrajPlanTurnAround=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround) ACC(v_max_turnAround,0,max([0 PosCircle1(1)-pos_s+4+l_veh]),speed,wait_turnAround)]);
    else
        if dec_trunAround==1
            a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
        else
            if PosCircle1(1)-pos_s>2*l_veh
                a_soll_TrajPlanTurnAround=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
            else
                a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
            end
        end
    end
    a_soll_TrajPlanTurnAround=min([a_soll_TrajPlanTurnAround,a_soll]);
elseif TypeOfTurnAround==2
    % 二次顺车掉头速度规划
    %     if TurnAroundState==0
    if TurnAroundState==0 || (TurnAroundState==1 && pos_s<=pos_start(1))
        if wait_turnAround==1 || wait_TrafficLight==1
            a_soll_TrajPlanTurnAround=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround) ACC(v_max_turnAround,0,max([0 pos_start(1)-pos_s+4+l_veh]),speed,wait_turnAround)]);
        else
            if dec_trunAround==1
                a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
            else
                if pos_start(1)-pos_s>2*l_veh
                    a_soll_TrajPlanTurnAround=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
                else
                    a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
                end
            end
        end
        dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_start(2)-PosCircle1(2)),(pos_start(1)-PosCircle1(1))))*TurningRadius+pos_start(1)-pos_s;
        a_soll_TrajPlanTurnAround=min(ACC(v_max,0,dis2pos_mid1+9,speed,0),a_soll_TrajPlanTurnAround);
        a_soll_TrajPlanTurnAround
    elseif TurnAroundState==1
        dis2pos_mid1=(atan2((pos_mid1(2)-PosCircle1(2)),(pos_mid1(1)-PosCircle1(1)))-atan2((pos_l-PosCircle1(2)),(pos_s-PosCircle1(1))))*TurningRadius;
        a_soll_TrajPlanTurnAround=min([ACClowSpeed(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed) ACClowSpeed(v_max_turnAround,0,dis2pos_mid1+9,speed)]);
        if sqrt((pos_mid1(1)-pos_s)^2+(pos_mid1(2)-pos_l)^2)<0.15
            a_soll_TrajPlanTurnAround=-4*sign(speed);
        end
        a_soll_TrajPlanTurnAround
    elseif TurnAroundState==2
        dis2pos_mid2=abs(atan2((pos_mid2(2)-PosCircle2(2)),(pos_mid2(1)-PosCircle2(1)))-atan2((pos_l-PosCircle2(2)),(pos_s-PosCircle2(1))))*TurningRadius;
        a_soll_TrajPlanTurnAround=min([ACClowSpeed(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed) ACClowSpeed(v_max_turnAround,0,dis2pos_mid2+9,speed)]);
        if sqrt((pos_mid2(1)-pos_s)^2+(pos_mid2(2)-pos_l)^2)<0.15
            a_soll_TrajPlanTurnAround=-4*sign(speed);
        end
    elseif TurnAroundState==3
        a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
    end
    a_soll_TrajPlanTurnAround=min([a_soll_TrajPlanTurnAround,a_soll]);
end

% 一次顺车掉头轨迹生成
if TypeOfTurnAround==1
    if pos_s>PosCircle1(1)-TurningRadius
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
            targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
            if targetSpeed==0
                adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
            else
                adavancedPerimeter=(targetSpeed+speed)*t_count_1/2;
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
    if traj_psi(1)==-90 && pos_s<PosCircle1(1)-4
        TurnAroundActive=0;
    end
    if traj_psi(80)~=90 && pos_s>PosCircle1(1)-TurningRadius
        a_soll_TrajPlanTurnAround=100;
    end
end
% 二次顺车掉头轨迹生成
if TypeOfTurnAround==2
    if TurnAroundState==1||TurnAroundState==0
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
            targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
            if targetSpeed==0
                adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
            else
                adavancedPerimeter=(targetSpeed+speed)*t_count_1/2;
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
    elseif TurnAroundState==2
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
    % if traj_psi(1)==-90 && (TurnAroundState==0)
    %     TurnAroundActive=0;
    % end
    if traj_psi(80)~=90 && TurnAroundState~=0
        a_soll_TrajPlanTurnAround=100;
    end
    
end

end
% if pos_s>PosCircle1(1)-TurningRadius
%     if pos_s-PosCircle1(1)>0
%         passedAngle=atan((pos_s-PosCircle1(1))/(PosCircle1(2)-pos_l));
%         passedPerimeter=passedAngle*TurningRadius;
%     else
%         passedPerimeter=pos_s-PosCircle1(1);
%     end
% end
% for count_1=1:1:80
%     t_count_1=0.05*count_1;
%
%
%     if traj_vs(count_1)==0
%         adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps);
%     else
%         adavancedPerimeter=(traj_vs(count_1)+speed)*t_count_1/2;
%     end
%     targetPerimeter=adavancedPerimeter+passedPerimeter;
%     targetAngle=targetPerimeter/TurningRadius;
%     if targetAngle<=0
%         traj_s(count_1)=pos_s+adavancedPerimeter;
%         traj_l(count_1)=pos_l;
%         traj_psi(count_1)=90;
%         targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%         traj_vs(count_1)=targetSpeed;
%         traj_vl(count_1)=0;
%         traj_omega(count_1)=0;
%     elseif targetAngle<=PI()
%         traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
%         traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
%         traj_psi(count_1)=90-targetAngle*180/PI();
%         targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%         traj_vs(count_1)=targetSpeed*cos(targetAngle);
%         traj_vl(count_1)=targetSpeed*sin(targetAngle);
%         traj_omega(count_1)=targetSpeed/TurningRadius*180/PI();
%     else
%         traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*PI());
%         traj_l(count_1)=PosCircle1(2)+TurningRadius;
%         traj_psi(count_1)=-90;
%         targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%         traj_vs(count_1)=-targetSpeed;
%         traj_vl(count_1)=0;
%         traj_omega(count_1)=0;
%     end
% end
