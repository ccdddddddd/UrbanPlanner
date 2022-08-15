function [NumRefLaneTurnAround,SRefLaneTurnAround,LRefLaneTurnAround,SEnd]=PathPlanTurnAroundDecider(LaneCenterlineTargetLane,PosCircle1,PosCircle2,TurningRadius,pos_s)
% 调用时LaneCenterline(TargetLaneIndexOpposite)代替LaneCenterlineTargetLane

NumRefLaneTurnAroundBefore=round((PosCircle1(1)-pos_s)/5);
SRefLaneTurnAroundBefore=linspace(pos_s,PosCircle1(1)-(PosCircle1(1)-pos_s)/NumRefLaneTurnAroundBefore,NumRefLaneTurnAroundBefore);
LRefLaneTurnAroundBefore=linspace(0,0,NumRefLaneTurnAroundBefore);

NumRefLaneTurnAroundDuring1=round((TurningRadius*0.5*pi)/0.5);
SRefLaneTurnAroundDuring1=zeros([1 NumRefLaneTurnAroundDuring1],'double');
LRefLaneTurnAroundDuring1=zeros([1 NumRefLaneTurnAroundDuring1],'double');
for i=1:1:NumRefLaneTurnAroundDuring1
    targetAngle=-pi/2+(i-1)*pi/2/NumRefLaneTurnAroundDuring1;
    SRefLaneTurnAroundDuring1(i)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
    LRefLaneTurnAroundDuring1(i)=PosCircle1(2)+sin(targetAngle)*TurningRadius;
end

NumRefLaneTurnAroundDuring2=round((PosCircle2(2)-PosCircle1(2))/0.5);
SRefLaneTurnAroundDuring2=[];
LRefLaneTurnAroundDuring2=[];
if NumRefLaneTurnAroundDuring2
    SRefLaneTurnAroundDuring2=linspace(PosCircle1(1)+TurningRadius,PosCircle2(1)+TurningRadius,NumRefLaneTurnAroundDuring2);
    LRefLaneTurnAroundDuring2=linspace(PosCircle1(2),PosCircle2(2)-(PosCircle2(2)-PosCircle1(2))/NumRefLaneTurnAroundDuring2,NumRefLaneTurnAroundDuring2);
end

NumRefLaneTurnAroundDuring3=round((TurningRadius*0.5*pi)/0.5);
SRefLaneTurnAroundDuring3=zeros([1 NumRefLaneTurnAroundDuring3],'double');
LRefLaneTurnAroundDuring3=zeros([1 NumRefLaneTurnAroundDuring3],'double');
for i=1:1:NumRefLaneTurnAroundDuring3
    targetAngle=(i-1)*pi/2/NumRefLaneTurnAroundDuring3;
    SRefLaneTurnAroundDuring3(i)=PosCircle1(1)+cos(targetAngle)*TurningRadius;
    LRefLaneTurnAroundDuring3(i)=PosCircle2(2)+sin(targetAngle)*TurningRadius;
end

NumRefLaneTurnAroundTransition=round((PosCircle2(2)+TurningRadius-LaneCenterlineTargetLane)/0.1);
SRefLaneTurnAroundTransition=[];
LRefLaneTurnAroundTransition=[];
if NumRefLaneTurnAroundTransition
    SRefLaneTurnAroundTransition=linspace(PosCircle2(1),PosCircle2(1)-NumRefLaneTurnAroundTransition+1,NumRefLaneTurnAroundTransition);
    LRefLaneTurnAroundTransition=linspace(PosCircle2(2)+TurningRadius,LaneCenterlineTargetLane+0.1,NumRefLaneTurnAroundTransition);
end

NumRefLaneTurnAroundAfter=round(20/5)+1;
SRefLaneTurnAroundAfter=linspace(PosCircle2(1)-NumRefLaneTurnAroundTransition,PosCircle2(1)-NumRefLaneTurnAroundTransition-20,NumRefLaneTurnAroundAfter);
LRefLaneTurnAroundAfter=linspace(LaneCenterlineTargetLane,LaneCenterlineTargetLane,NumRefLaneTurnAroundAfter);

SRefLaneTurnAround=[SRefLaneTurnAroundBefore SRefLaneTurnAroundDuring1 SRefLaneTurnAroundDuring2 SRefLaneTurnAroundDuring3 SRefLaneTurnAroundTransition SRefLaneTurnAroundAfter];
LRefLaneTurnAround=[LRefLaneTurnAroundBefore LRefLaneTurnAroundDuring1 LRefLaneTurnAroundDuring2 LRefLaneTurnAroundDuring3 LRefLaneTurnAroundTransition LRefLaneTurnAroundAfter];

NumRefLaneTurnAround=length(SRefLaneTurnAround);
if NumRefLaneTurnAround<100
    SRefLaneTurnAround=[SRefLaneTurnAround zeros([1 100-NumRefLaneTurnAround],'double')];
    LRefLaneTurnAround=[LRefLaneTurnAround zeros([1 100-NumRefLaneTurnAround],'double')];
end
SEnd=PosCircle2(1)-NumRefLaneTurnAroundTransition; % 过渡结束位置的s坐标

% figure;
% plot(SRefLaneTurnAroundBefore,LRefLaneTurnAroundBefore,'ro');
% hold on;
% plot(SRefLaneTurnAroundDuring1,LRefLaneTurnAroundDuring1,'b*');
% hold on;
% plot(SRefLaneTurnAroundDuring2,LRefLaneTurnAroundDuring2,'r+');
% hold on;
% plot(SRefLaneTurnAroundDuring3,LRefLaneTurnAroundDuring3,'gs');
% hold on;
% plot(SRefLaneTurnAroundTransition,LRefLaneTurnAroundTransition,'cd');
% hold on;
% plot(SRefLaneTurnAroundAfter,LRefLaneTurnAroundAfter,'bl>');
% axis equal;
% figure;
% plot(SRefLaneTurnAround,LRefLaneTurnAround,'b*');
% axis equal;

end
% PathPlanTurnAroundDecider(10.5,[0,5],[0,6],5,-50)