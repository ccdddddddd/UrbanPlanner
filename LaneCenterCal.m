function LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite)

CurrentLane=-CurrentLane;
WidthOfLanes=[WidthOfLaneCurrent WidthOfGap WidthOfLanesOpposite];
Lane_boundary=zeros(length(WidthOfLanes),3);
LaneCenterline=zeros(1,7);
for i=1:1:length(WidthOfLanes)
%     if i==1
%         Lane_boundary(i,1)=WidthOfLanes(length(WidthOfLanes));
%     else
%         Lane_boundary(i,1)=Lane_boundary(i-1,1)+WidthOfLanes(length(WidthOfLanes)+1-i);
%     end
    Lane_boundary(i,1)=sum(WidthOfLanes(1:i));
    Lane_boundary(i,3)=i+(CurrentLane-1);
end
Lane_boundary(:,2)=Lane_boundary(:,1)+(WidthOfLanes(1)*0.5+pos_l_CurrentLane-Lane_boundary(1,1));
for j=1:1:length(WidthOfLanes)
    Lane_boundary(j,1)=Lane_boundary(j,2)-WidthOfLanes(j);
end
allLaneCenterline=0.5*(Lane_boundary(:,2)+Lane_boundary(:,1));
LaneCenterline(1,1:length([(allLaneCenterline(3:2+NumOfLanesOpposite))' allLaneCenterline(1)]))=[(allLaneCenterline(3:2+NumOfLanesOpposite))' allLaneCenterline(1)];
end

% WidthOfLaneCurrent=3.1;
% WidthOfLanesOpposite=[3.2 3.3 3.4 3.5 3.6 0];
% WidthOfGap=1;
% CurrentLane=1;
% pos_l_CurrentLane=0;
% NumOfLanesOpposite=5;
% LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite)
