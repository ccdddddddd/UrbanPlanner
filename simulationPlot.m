clc
clear
close all
%换道
%换道
CalibrationVars.a_lateral=3;
CalibrationVars.wComfort2EfficiencyLaneChange=0.5; % Jchg=w*(aLst/aMax).^2+(1-w)*(xEnd/xEndMax).^2
CalibrationVars.tMaxLaneChange=6;
CalibrationVars.aMaxLaneChange=1.5;
CalibrationVars.aMinLaneChange=-2;
CalibrationVars.vMaxLaneChange=20;
CalibrationVars.vMinLaneChange=6;
CalibrationVars.decel=-4;
CalibrationVars.speedGap=2;
CalibrationVars.d_max=60;
CalibrationVars.d_safe=5;
CalibrationVars.t_re=0.5;
CalibrationVars.MovingRoomFromCenterLane=1;
CalibrationVars.checkTimeGap=0.1;
CalibrationVars.wDis=0.5;
CalibrationVars.wAlat=1-CalibrationVars.wDis;
CalibrationVars.linspaceNum=50;
CalibrationVars.linspaceNumCrossCal=25;
CalibrationVars.linspaceNumALateralCal=10;
CalibLCPath=CalibrationVars;
clearvars CalibrationVars
%重规划
CalibrationVars.ds=1; % 最大步数
CalibrationVars.numOfMaxSteps=round(100/CalibrationVars.ds); % 最大步数
CalibrationVars.wOffset= 1;
CalibrationVars.wOffsetDer= 50; 
CalibrationVars.wOffsetSecDer= 1000; 
CalibrationVars.wOffsetThrDer= 50000; 
CalibrationVars.FLAGS_lateral_derivative_bound_default=2;
CalibrationVars.MovingRoomFromCenterLane=1;
CalibrationVars.steerAngleRateRatioMax=2;
CalibrationVars.steerAngleRateRatioMin=1.5;
CalibrationVars.stepByOffsetBound=0.5;
CalibReplanPath=CalibrationVars;
clearvars CalibrationVars
%mcts
CalibrationVars.numOfMaxSteps=10;
CalibrationVars.accel=2;
CalibrationVars.decel=-3;
CalibrationVars.stepLength=0.5; % s
CalibrationVars.gamma=0.8; % TBD
CalibrationVars.numOfIteration=5000;
CalibrationVars.a_lateral=3;
CalibrationVars.v_min_decel=0;
CalibrationVars.p_actionTime=0.5;

CalibrationVars.d_safe=1;
CalibrationVars.d_max=60;

CalibrationVars.t_re=0.5;
CalibrationVars.epsilonSwitch=int16(5);
CalibrationVars.UCBswitch=1;
CalibrationVars.UCBconstant=1;
CalibrationVars.debugFlag=false;
CalibrationVars.w1=0.7; % 距离权重
CalibrationVars.w2=1-CalibrationVars.w1; % 速度权重
CalibMCTS=CalibrationVars;
clearvars CalibrationVars
%qp
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
CalibrationVars.jMax=70;
CalibrationVars.jMin=-70;
CalibrationVars.offsetMax=10; % 与MCTS生成轨迹的最大可接受偏差
CalibrationVars.numOfMaxSteps=50; %50; % 最大步数
CalibrationVars.wOffset=0.5; % 0.5; 对角线上元素的值为wOffset
CalibrationVars.wJerk=1-CalibrationVars.wOffset; % 0.5; % 对角线上元素的值为wJerk
CalibQP=CalibrationVars;
clearvars CalibrationVars
CalibrationVars.urbanPlanner.pathBuffer = 1;
CalibrationVars.CalibLCPath = CalibLCPath;
CalibrationVars.CalibReplanPath = CalibReplanPath;
CalibrationVars.CalibMCTS = CalibMCTS;
CalibrationVars.CalibQP = CalibQP;
%全局
GlobVars.urbanPlanner.ischanginglanes = 0;
GlobVars.urbanPlanner.isreplanPath = 0;
GlobVars.urbanPlanner.changLaneStart_s = 0;
GlobVars.urbanPlanner.changLaneEnd_s = 0;
GlobVars.urbanPlanner.changLanePara = 0;
GlobVars.urbanPlanner.replanStart_s = 0;
GlobVars.urbanPlanner.replanEnd_s = 0;
GlobVars.urbanPlanner.replanLSequcence = 0;
GlobVars.urbanPlanner.curTargetLaneIndex = 0;
GlobVars.urbanPlanner.laneChangeDirection = 0;

%静态入参
Parameter.w_veh=double(1.8); %车宽
Parameter.l_veh=double(5); %车长
Parameter.turningRadius = 7;
Parameter.widthOfVehicle=1.8;
Parameter.widthOfVehicle=2;
Parameter.maxSteerAngle=8.20304748437;
Parameter.steerRatio=16;
Parameter.wheelBase=2.8448;
Parameter.maxSteerAngleRate=6.98131700798;


waypoints = [0 0; 50 20; 100 0; 150 10];
refPath = referencePathFrenet(waypoints);

show(refPath)
axis equal

pathStates = refPath.interpolate(linspace(0,refPath.PathLength,round(refPath.PathLength/5)));
wayPointsLeft_frenet = zeros(length(pathStates(:,6)),6);%[s,ds,dds,l,dl,ddl]
wayPointsLeft_frenet(:,1) = pathStates(:,6);
wayPointsLeft_frenet(:,4) = wayPointsLeft_frenet(:,4)+3.2;
wayPointsLeft_global = frenet2global(refPath,wayPointsLeft_frenet);%[x,y,theta,k,speed,a]
refPathLeft = referencePathFrenet(wayPointsLeft_global(:,1:2));

hold on
show(refPathLeft)


%实时入参
arraySize = [50, 1];
ObjectsInfo.objectsNum = 0; %目标物数量
objects.s = zeros(51,1); %frenet 坐标
objects.l = zeros(51,1);%frenet 坐标
objects.psi = zeros(51,1);%在frenet坐标系横摆角
objects.speed = zeros(51,1);
objects.timestamp = zeros(51,1);
objects.length = 0; %目标物长度
objects.width = 0; %目标物宽度
ObjectsInfo.objects = repmat(objects, arraySize);

BasicsInfo.speed = 0; %车速
BasicsInfo.acce = 0;
BasicsInfo.v_max = 50/3.6; %期望车速
BasicsInfo.s = 0; %frenet 坐标
BasicsInfo.l = 0.5; %frenet 坐标
BasicsInfo.psi = 0; %frenet 坐标
BasicsInfo.currentLaneIndex = 2; %当前车道序号

BasicsInfo.targetLaneIndex = 2; %目标车道序号
BasicsInfo.d_veh2goal = 200; %与终点距离
BasicsInfo.goalLaneIndex = 1; %终点车道

BasicsInfo.trafficLightPhase = [30,-2,-2,-40];%信号灯相位
BasicsInfo.d_veh2stopline = 200; %停止线距离
BasicsInfo.d_veh2cross = 200; %与人行道距离
BasicsInfo.d_veh2waitingArea = 200; %与待转区停止线距离

ReferenceLineInfo.currentLane.sSequcence = [0,30,200];
ReferenceLineInfo.currentLane.kSequcence = [0,0,0];
ReferenceLineInfo.currentLane.v_maxSequcence = [50/3.6,50/3.6,50/3.60];
% ReferenceLineInfo.currentLane.rightSideline_s = 
% ReferenceLineInfo.currentLane.rightSideline_l = 
ReferenceLineInfo.rightLane.s = [0,30,200];
ReferenceLineInfo.rightLane.l = [-3.2,-3.2,-3.2];
ReferenceLineInfo.rightLane.k = [0,0,0];
ReferenceLineInfo.leftLane.s = [0,30,200];
ReferenceLineInfo.leftLane.l = [3.2,3.2,3.2];
ReferenceLineInfo.leftLane.k = [0,0,0];

ReferenceLineInfo.currentLane.sSequcence = linspace(0,refPath.PathLength,20);
ReferenceLineInfo.currentLane.kSequcence = refPath.curvature(linspace(0,refPath.PathLength,20)')';
ReferenceLineInfo.currentLane.v_maxSequcence = zeros(1,20)+50/3.6;
ReferenceLineInfo.leftLane.s = wayPointsLeft_frenet(:,1);
ReferenceLineInfo.leftLane.l = wayPointsLeft_frenet(:,4);
ReferenceLineInfo.leftLane.k = wayPointsLeft_global(:,4);
% ReferenceLineInfo.currentLane.sSequcence = pathStates(:,6)';
% ReferenceLineInfo.currentLane.kSequcence = pathStates(:,4)';
% ReferenceLineInfo.currentLane.v_maxSequcence = zeros(1,size(pathStates,1))+50/3.6;

hold on
for i=1:1000
i
if i==86
i
end
% 
% if i==10
% BasicsInfo.targetLaneIndex = 1; %目标车道序号
% end
% if i>11 && GlobVars.urbanPlanner.ischanginglanes == 0
% BasicsInfo.currentLaneIndex = 1;
% refPath = refPathLeft;
% end
if i~=1
% %参考线选择
if i==50
BasicsInfo.targetLaneIndex = 1; %目标车道序号
end
if i>51 && GlobVars.urbanPlanner.ischanginglanes == 0
BasicsInfo.currentLaneIndex = 1;
refPath = refPathLeft;
end

%坐标转换 global2frenet
vehstate_frenet = global2frenet(refPath,globalTraj(2,:));
BasicsInfo.speed = sqrt(vehstate_frenet(2)^2+vehstate_frenet(2)^2*vehstate_frenet(5)^2); %车速
BasicsInfo.acce = sqrt(vehstate_frenet(3)^2+vehstate_frenet(3)^2*vehstate_frenet(6)^2);
BasicsInfo.s = vehstate_frenet(1); %frenet 坐标
BasicsInfo.l = vehstate_frenet(4); %frenet 坐标
BasicsInfo.psi = atand(vehstate_frenet(5)); %frenet 坐标
end
%调用
tic
[traj,trajectoryType,GlobVars] = urbanplanner(BasicsInfo,ObjectsInfo,ReferenceLineInfo,GlobVars,Parameter,CalibrationVars);
planningstatedisp(trajectoryType);
toc
% BasicsInfo.speed = sqrt(traj.s_dot(2)^2+traj.l_dot(2)^2); %车速
% BasicsInfo.acce = sqrt(traj.s_ddot(2)^2+traj.l_ddot(2)^2);
% BasicsInfo.s = traj.s(2); %frenet 坐标
% BasicsInfo.l = traj.l(2); %frenet 坐标
% BasicsInfo.psi = atand(traj.dl(2)); %frenet 坐标
%坐标转换 frenet2global
Trajectory = [traj.s',traj.s_dot',traj.s_ddot',traj.l',traj.dl',traj.ddl'];
frenetTraj.Trajectory = Trajectory;
frenetTraj.Times = (0:0.1:(length(traj.s)-1)/10)';
globalTraj = frenet2global(refPath,frenetTraj.Trajectory);
%画图
if exist('p_v','var') ==1
    delete(p_v);
    delete(p_traj);
end
p_v=plot(globalTraj(1,1),globalTraj(1,2),'co');
p_traj = plot(globalTraj(:,1),globalTraj(:,2),'b.');
pause(0.01)
end
%状态显示
function planningstatedisp(trajectoryType)
switch trajectoryType
    case 1
        disp('换道中+偏离+位于目标车道+重规划目标车道+速度规划成功')
    case -1
        disp('换道中+偏离+位于目标车道+重规划目标车道+速度规划失败（紧急制动）')
    case 2
        disp('换道中+偏离+位于原车道+重换道成功+速度规划成功')
    case 3
        disp('换道中+偏离+位于原车道+重换道失败+重规划原车道+速度规划成功')
    case -3
        disp('换道中+偏离+位于原车道+重换道失败+重规划原车道+速度规划失败（紧急制动）')
    case 4
        disp('换道中+速度规划成功')
    case -4
        disp('换道中+速度规划失败+位于目标车道（紧急制动）')
    case 5
        disp('换道中+速度规划失败+位于原车道+重换道成功+速度规划成功')
    case 6
        disp('换道中+速度规划失败+位于原车道+重换道失败+重规划原车道+速度规划成功')
    case -6
        disp('换道中+速度规划失败+位于原车道+重换道失败+重规划原车道+速度规划失败（紧急制动）')
    case 7
        disp('重规划目标车道中+速度规划成功')
    case -7
        disp('重规划目标车道中+速度规划失败（紧急制动）')
    case 8
        disp('需换道+换道规划成功+速度规划成功')
    case 9
        disp('（无需换道或换道不ok）重规划原车到中+速度规划成功')
    case -9
        disp('（无需换道或换道不ok）重规划原车到中+速度规划失败（紧急制动）')
    case 10
        disp('（无需换道或换道不ok且需重规划）重规划原车道+速度规划成功')
    case -10
        disp('（无需换道或换道不ok且需重规划）重规划原车道+速度规划失败（紧急制动）')
    case 11
        disp('（无需换道或换道不ok且无需重规划）+速度规划成功')
    case -11
        disp('（无需换道或换道不ok且无需重规划）+速度规划失败（紧急制动）')
end
end