function [TargetLaneIndex,BackupTargetLaneIndex]=LaneSelectionWithBlockedLanes(NumOfLanes,LanesWithFail,TargetLaneIndex,CurrentLaneIndex)
% NumOfLanes=2; % 车道数量
% LanesWithFail=[1 2]; % 故障车所在车道序号，车道序号自小到大排列，最左侧车道序号为1
% TargetLaneIndex=2; % 原目标车道
% CurrentLaneIndex=1; 本车所在车道
LaneInfos=zeros(NumOfLanes,2); % Failfalg Dis2Tar+Dis2Cur
for i=1:1:length(LanesWithFail)
    LaneInfos(LanesWithFail(i),1)=1;
end
for i=1:1:NumOfLanes
    LaneInfos(i,2)=abs(i-TargetLaneIndex)+abs(i-CurrentLaneIndex);
    %     LaneInfos(i,2)=abs(i-TargetLaneIndex);
    %     LaneInfos(i,3)=abs(i-CurrentLaneIndex);
end
Neighbor2Current=[];
if isempty(LaneInfos((LaneInfos(:,1)==0),2))==0
    Neighbor2Current=find((LaneInfos(:,2))==min(LaneInfos((LaneInfos(:,1)==0),2)) & LaneInfos(:,1)==0);
end
% Neighbor2Current=[];
% if isempty(Neighbor2Target)==0
%     if length(Neighbor2Target)==2
%         if LaneInfos(Neighbor2Target(1),3)> LaneInfos(Neighbor2Target(2),3)
%             Neighbor2Current=Neighbor2Target(2);
%         elseif LaneInfos(Neighbor2Target(1),3)< LaneInfos(Neighbor2Target(2),3)
%             Neighbor2Current=Neighbor2Target(1);
%         else
%             Neighbor2Current=Neighbor2Target;
%         end
%     elseif length(Neighbor2Target)==1
%         Neighbor2Current=Neighbor2Target;
%     end    
% end
BackupTargetLaneIndex=-1;
if isempty(Neighbor2Current)==0
    if length(Neighbor2Current)==2 || length(Neighbor2Current)==1
        TargetLaneIndex=Neighbor2Current(1);
    end
    if length(Neighbor2Current)==2
        BackupTargetLaneIndex=Neighbor2Current(2);
    end
end
if BackupTargetLaneIndex==-1
    if TargetLaneIndex>CurrentLaneIndex && CurrentLaneIndex-1>=1 
        if LaneInfos(CurrentLaneIndex-1,1)==0
            BackupTargetLaneIndex=CurrentLaneIndex-1;
        end
    end
end
if BackupTargetLaneIndex==-1
    if TargetLaneIndex<CurrentLaneIndex && CurrentLaneIndex+1<=NumOfLanes 
        if LaneInfos(CurrentLaneIndex+1,1)==0
            BackupTargetLaneIndex=CurrentLaneIndex+1;
        end
    end
end 
if TargetLaneIndex~=1
    TargetLaneIndex
end

end
% TargetLaneIndex 现目标车道
% BackupTargetLaneIndex % 现备用目标车道，仅当车辆左右换道都可以避开故障车时存在（此时TargetLaneIndex为左侧车道，BackupTargetLaneIndex为右侧车道），否则为-1
