function LaneIndexOfPoint=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,traj_y)

CurrentLane=-CurrentLane;
WidthOfLanes=[WidthOfLaneCurrent WidthOfGap WidthOfLanesOpposite];
Lane_boundary=zeros(length(WidthOfLanes),3);
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
% CurrentTargetLaneIndex1=-1;
if traj_y>=Lane_boundary(1,2)-WidthOfLanes(1) && traj_y<Lane_boundary(length(WidthOfLanes),2)
    LaneIndexOfPoint=Lane_boundary((Lane_boundary(:,2)-traj_y)>=0,3);
    LaneIndexOfPoint=LaneIndexOfPoint(1);
elseif traj_y<Lane_boundary(1,2)-WidthOfLanes(1)
    LaneIndexOfPoint=Lane_boundary(1,3)-1;
elseif traj_y>=Lane_boundary(length(WidthOfLanes),2)
    LaneIndexOfPoint=double(NumOfLanesOpposite);
else
    LaneIndexOfPoint=0;%20220324
end
end

% WidthOfLaneCurrent=3.1;
% WidthOfLanesOpposite=[3.2 3.3 3.4 3.5 3.6 0];
% WidthOfGap=1;
% CurrentLane=1;
% pos_l_CurrentLane=0;
% NumOfLanesOpposite=5;
% traj_y=-1.5;
% LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,traj_y)
