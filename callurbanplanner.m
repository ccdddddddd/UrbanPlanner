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
BasicsInfo.l = 0; %frenet 坐标
BasicsInfo.psi = 0; %frenet 坐标
BasicsInfo.currentLaneIndex = 1; %当前车道序号

BasicsInfo.targetLaneIndex = 1; %目标车道序号
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
CalibrationVars.FLAGS_lateral_derivative_bound_default=2;
CalibrationVars.MovingRoomFromCenterLane=1;
CalibrationVars.steerAngleRateRatioMax=2;
CalibrationVars.steerAngleRateRatioMin=1.5;
CalibrationVars.stepByOffsetBound=0.5;
CalibReplanPath=CalibrationVars;
clearvars CalibrationVars
%mcts
CalibrationVars.numOfMaxSteps=10;
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
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
CalibrationVars.debugFlag=false;
CalibrationVars.w1=0.7; % 距离权重
CalibrationVars.w2=1-CalibrationVars.w1; % 速度权重
CalibMCTS=CalibrationVars;
clearvars CalibrationVars
%qp
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
CalibrationVars.jMax=10;
CalibrationVars.jMin=-10;
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

[traj,GlobVars] = urbanplanner(BasicsInfo,ObjectsInfo,ReferenceLineInfo,GlobVars,Parameter,CalibrationVars);
