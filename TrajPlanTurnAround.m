function [a_soll_TrajPlanTurnAround,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,s_circle1,l_circle1,s_circle2,l_circle2,dec_trunAround,wait_turnAround,TargetLaneIndexOpposite,TypeOfTurnAround,TurnAroundActive]=...,
    TrajPlanTurnAround(CurrentLaneFrontDis,CurrentLaneFrontVel,TurningRadius,speed,pos_l_CurrentLane,pos_s,pos_l,NumOfLanesOpposite,WidthOfLanesOpposite,WidthOfGap,WidthOfLaneCurrent,s_turnaround_border,...,
    s_circle1,l_circle1,s_circle2,l_circle2,dec_trunAround,wait_turnAround,IndexOfLaneOppositeCar,SpeedOppositeCar,PosSOppositeCar,v_max,TypeOfTurnAround,a_soll,wait_TrafficLight,...,
    TargetLaneIndexOpposite,TurnAroundActive)

w_veh=1.8;
l_veh=5;
D_safe=0.5;
dec2line=0.2;
a_min=-3;
a_max_com=1.5;
v_max_turnAround=5;
d_veh2cross=zeros([5,1]);
traj_s=zeros([1 80]);
traj_l=zeros([1 80]);
traj_psi=zeros([1 80]);
traj_vs=zeros([1 80]);
traj_vl=zeros([1 80]);
traj_omega=zeros([1 80]);
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
        if d_cur2pot_tar>2*TurningRadius
            d_cur2tar=d_cur2pot_tar;
            TargetLaneIndexOpposite=i;
            TypeOfTurnAround=1;
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        elseif 0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar>2*TurningRadius
            d_cur2tar=0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar;
            TargetLaneIndexOpposite=i;
            TypeOfTurnAround=1;
            pos_l_TargetLane=pos_l_CurrentLane+d_cur2tar;
            break;
        end
    end
    % 一次顺车掉头路径生成
    if TypeOfTurnAround==1
        s_circle1=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe;
        l_circle1=pos_l_CurrentLane+TurningRadius;
        % s_start=s_circle1;
        s_circle2=s_turnaround_border-TurningRadius-0.5*w_veh-D_safe;
        l_circle2=pos_l_TargetLane-TurningRadius;
    end
end
%筛选一次顺车掉头前车后车到路径圆心距离
if TypeOfTurnAround==1
    IndexOfLaneOppositeCarFront=zeros([6,1]);
    SpeedOppositeCarFront=zeros([6,1]);
    PosSOppositeCarFront=zeros([6,1]);
    PosSOppositeCarFront=PosSOppositeCarFront+200;
    PosSOppositeCarRear=zeros([6,1]);
    PosSOppositeCarRear=PosSOppositeCarRear-200;
    j=1;
    for i=1:length(IndexOfLaneOppositeCar)
        if i==1
            if PosSOppositeCar(i)-s_circle1>=0
                IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                PosSOppositeCarFront(j)=PosSOppositeCar(i)-s_circle1;
                SpeedOppositeCarFront(j)=SpeedOppositeCar(i);
            else
                IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                PosSOppositeCarRear(j)=PosSOppositeCar(i)-s_circle1;
            end
        else
            if IndexOfLaneOppositeCar(i-1)==IndexOfLaneOppositeCar(i)
                if PosSOppositeCar(i)-s_circle1>=0&&PosSOppositeCar(i)-s_circle1<PosSOppositeCarFront(j)
                    IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                    PosSOppositeCarFront(j)=PosSOppositeCar(i)-s_circle1;
                    SpeedOppositeCarFront(j)=SpeedOppositeCar(i);
                elseif PosSOppositeCar(i)-s_circle1<0&&PosSOppositeCar(i)-s_circle1>PosSOppositeCarRear(j)
                    IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                    PosSOppositeCarRear(j)=PosSOppositeCar(i)-s_circle1;
                end
            else
                j=j+1;
                if PosSOppositeCar(i)-s_circle1>=0
                    IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                    PosSOppositeCarFront(j)=PosSOppositeCar(i)-s_circle1;
                    SpeedOppositeCarFront(j)=SpeedOppositeCar(i);
                else
                    IndexOfLaneOppositeCarFront(j)=IndexOfLaneOppositeCar(i);
                    PosSOppositeCarRear(j)=PosSOppositeCar(i)-s_circle1;
                end
            end
        end
    end
end
% 一次顺车掉头决策
if TypeOfTurnAround==1 %&& pos_s<s_circle1
    for i=1:TargetLaneIndexOpposite
        if 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))<TurningRadius
            d_veh2cross(i)=s_circle1-pos_s+TurningRadius*acos((TurningRadius-(0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))))/TurningRadius);
        elseif 0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))>TurningRadius+l_circle2-l_circle1
            d_veh2cross(i)=s_circle1-pos_s+(TurningRadius*asin((0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))-TurningRadius-l_circle2+l_circle1)/TurningRadius)+l_circle2-l_circle1+TurningRadius*pi/2);
        else
            d_veh2cross(i)=s_circle1-pos_s+TurningRadius*pi/2+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1))-TurningRadius;
        end
%         d_veh2cross(i)=s_circle1-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确
    end
    % 策略模式判断
    d_bre=(0-speed.^2)/(2*a_min);
    if dec_trunAround==0
        if s_circle1-pos_s<=d_bre+2*l_veh && s_circle1-pos_s>0 && pos_l<l_circle1
            dec_trunAround=1;
        end
    else
        if s_circle1-pos_s<=0 || wait_turnAround==1 || pos_l>l_circle1%20220225
            dec_trunAround=0;
        end
    end
    % 停车决策
    if dec_trunAround==1
%         for i=1:TargetLaneIndexOpposite
%             d_veh2cross(i)=s_circle1-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确
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
    if wait_turnAround==1 && s_circle1-pos_s<10
        wait_turnAround=0;
        a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0);
        for j=1:length(IndexOfLaneOppositeCarFront)
            if IndexOfLaneOppositeCarFront(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCarFront(j)>0
                timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh-TurningRadius)/max([SpeedOppositeCarFront(j) 0.00001])]);
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
    end
end
% ACC速度规划
if (wait_turnAround==1 && s_circle1>pos_s) || wait_TrafficLight==1
    % a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]);
    a_soll_TrajPlanTurnAround=min([ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround) ACC(v_max_turnAround,0,max([0 s_circle1-pos_s+4+l_veh]),speed,wait_turnAround)]);
else
    if dec_trunAround==1
    a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
    else
        if s_circle1-pos_s>2*l_veh
            a_soll_TrajPlanTurnAround=ACC(v_max,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
        else
            a_soll_TrajPlanTurnAround=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,wait_turnAround);
        end
    end
end
a_soll_TrajPlanTurnAround=min([a_soll_TrajPlanTurnAround,a_soll]);
% 一次顺车掉头轨迹生成 
if pos_s>s_circle1-TurningRadius
    if pos_s-s_circle1>0&&pos_l<l_circle1
        passedAngle=atan((pos_l-l_circle1)/(pos_s-s_circle1));
        passedPerimeter=(pi/2+passedAngle)*TurningRadius;
    elseif pos_l<=l_circle2&&pos_l>=l_circle1
        passedPerimeter=pos_l-l_circle1+TurningRadius*pi/2;
    elseif pos_l>l_circle2
        passedAngle=atan((pos_l-l_circle2)/(pos_s-s_circle2));
        if passedAngle<0
            passedPerimeter=l_circle2-l_circle1+TurningRadius*pi+s_circle2-pos_s;
        else
            passedPerimeter=passedAngle*TurningRadius+l_circle2-l_circle1+TurningRadius*pi/2;
        end
    else
        passedPerimeter=pos_s-s_circle1;
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
        elseif targetPerimeter<=pi*TurningRadius/2+l_circle2-l_circle1&&targetPerimeter>=pi*TurningRadius/2
            targetAngle=0;
        else
            targetAngle=(targetPerimeter-(l_circle2-l_circle1))/TurningRadius-pi/2;
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
            traj_s(count_1)=s_circle1+cos(targetAngle)*TurningRadius;
            traj_l(count_1)=l_circle1+sin(targetAngle)*TurningRadius;
            traj_psi(count_1)=-targetAngle*180/pi;
            traj_vs(count_1)=-targetSpeed*sin(targetAngle);
            traj_vl(count_1)=targetSpeed*cos(targetAngle);
            traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
        elseif targetAngle==0%PI()
            traj_s(count_1)=s_circle1+TurningRadius;
            traj_l(count_1)=l_circle1+targetPerimeter-pi*TurningRadius/2;
            traj_psi(count_1)=-targetAngle*180/pi;
            traj_vs(count_1)=0;
            traj_vl(count_1)=targetSpeed;
            traj_omega(count_1)=targetSpeed;
        elseif targetAngle<=pi/2%PI()
            traj_s(count_1)=s_circle1+cos(targetAngle)*TurningRadius;
            traj_l(count_1)=l_circle2+sin(targetAngle)*TurningRadius;
            traj_psi(count_1)=-targetAngle*180/pi;
            traj_vs(count_1)=-targetSpeed*sin(targetAngle);
            traj_vl(count_1)=targetSpeed*cos(targetAngle);
            traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi;
        else
            traj_s(count_1)=s_circle1-(targetPerimeter-TurningRadius*pi-l_circle2+l_circle1);
            traj_l(count_1)=l_circle2+TurningRadius;
            traj_psi(count_1)=-90;
            traj_vs(count_1)=-targetSpeed;
            traj_vl(count_1)=0;
            traj_omega(count_1)=0;
        end
    end
end
if traj_psi(1)==-90 && pos_s<s_circle1-4
    TurnAroundActive=0;
end
if traj_psi(80)~=90 && pos_s>s_circle1-TurningRadius
    a_soll_TrajPlanTurnAround=100;
end
end
% if pos_s>s_circle1-TurningRadius
%     if pos_s-s_circle1>0
%         passedAngle=atan((pos_s-s_circle1)/(l_circle1-pos_l));
%         passedPerimeter=passedAngle*TurningRadius;
%     else
%         passedPerimeter=pos_s-s_circle1;
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
%         traj_s(count_1)=s_circle1+cos(targetAngle)*TurningRadius;
%         traj_l(count_1)=l_circle1+sin(targetAngle)*TurningRadius;
%         traj_psi(count_1)=90-targetAngle*180/PI();
%         targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%         traj_vs(count_1)=targetSpeed*cos(targetAngle);
%         traj_vl(count_1)=targetSpeed*sin(targetAngle);
%         traj_omega(count_1)=targetSpeed/TurningRadius*180/PI();
%     else
%         traj_s(count_1)=s_circle1-(targetPerimeter-TurningRadius*PI());
%         traj_l(count_1)=l_circle1+TurningRadius;
%         traj_psi(count_1)=-90;
%         targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]);
%         traj_vs(count_1)=-targetSpeed;
%         traj_vl(count_1)=0;
%         traj_omega(count_1)=0;
%     end
% end
