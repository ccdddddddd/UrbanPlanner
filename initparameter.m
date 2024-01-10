%lanechange
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
CalibrationVars.nodeRef=1;
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
%urbanplanner
CalibrationVars.urbanPlanner.pathBuffer = 0.3;
CalibrationVars.urbanPlanner.pathPointSpace = 2;
CalibrationVars.urbanPlanner.maxLatAcce = 3;
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
