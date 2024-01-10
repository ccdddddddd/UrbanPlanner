function [ReferenceLineInfo,refPath] = referencelinegeneration(LaneLineInfo,v_max,CalibrationVars)
%CalibrationVars
pathPointSpace = CalibrationVars.urbanPlanner.pathPointSpace; %2;
maxLatAcce = CalibrationVars.urbanPlanner.maxLatAcce ;% 3;
%参考线生成
curWayPoints = zeros(length(LaneLineInfo.currentLanePointsX),2);
curWayPoints(:,1) = LaneLineInfo.currentLanePointsX';
curWayPoints(:,2) = LaneLineInfo.currentLanePointsY';
refPath = referencePathFrenet(curWayPoints);
numOfPathPoints = round(refPath.PathLength/pathPointSpace);
arclengths = linspace(0,refPath.PathLength,numOfPathPoints);
kSequcence = refPath.curvature(arclengths')';
ReferenceLineInfo.currentLane.sSequcence = arclengths;
ReferenceLineInfo.currentLane.kSequcence = kSequcence;
ReferenceLineInfo.currentLane.v_maxSequcence = max(sqrt(maxLatAcce./kSequcence),v_max);
if ~isempty(LaneLineInfo.leftLanePointsX)
    leftWayPoints = zeros(length(LaneLineInfo.leftLanePointsX),2);
    leftWayPoints(:,1) = LaneLineInfo.leftLanePointsX';
    leftWayPoints(:,2) = LaneLineInfo.leftLanePointsY';
    refPath_L = referencePathFrenet(leftWayPoints);
    arclengths = linspace(0,refPath_L.PathLength,numOfPathPoints);
    refPathPoints_L = interpolate(refPath_L,arclengths);%[x y theta kappa dkappa s]
    frenetState = refPath.global2frenet(refPathPoints_L);%[S dS ddS L dL ddL]
    ReferenceLineInfo.leftLane.s = frenetState(:,1)';
    ReferenceLineInfo.leftLane.l = frenetState(:,4)';
    ReferenceLineInfo.leftLane.dl = frenetState(:,5)';
    ReferenceLineInfo.leftLane.k = refPathPoints_L(:,4)';
else
    ReferenceLineInfo.leftLane.s = [];
    ReferenceLineInfo.leftLane.l = [];
    ReferenceLineInfo.leftLane.dl = [];
    ReferenceLineInfo.leftLane.k = [];
end
if ~isempty(LaneLineInfo.rightLanePointsX)
    rightWayPoints = zeros(length(LaneLineInfo.rightLanePointsX),2);
    rightWayPoints(:,1) = LaneLineInfo.rightLanePointsX';
    rightWayPoints(:,2) = LaneLineInfo.rightLanePointsY';
    refPath_R = referencePathFrenet(rightWayPoints);
    arclengths = linspace(0,refPath_R.PathLength,numOfPathPoints);
    refPathPoints_R = interpolate(refPath_R,arclengths);%[x y theta kappa dkappa s]
    frenetState = refPath.global2frenet(refPathPoints_R);%[S dS ddS L dL ddL]
    ReferenceLineInfo.rightLane.s = frenetState(:,1)';
    ReferenceLineInfo.rightLane.l = frenetState(:,4)';
    ReferenceLineInfo.rightLane.dl = frenetState(:,5)';
    ReferenceLineInfo.rightLane.k = refPathPoints_R(:,4)';
else
    ReferenceLineInfo.rightLane.s = [];
    ReferenceLineInfo.rightLane.l = [];
    ReferenceLineInfo.rightLane.dl = [];
    ReferenceLineInfo.rightLane.k = [];
end

end