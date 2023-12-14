
clear all
close all
% openfig('untitled.fig')
s_0=0;
speed=20;
CalibrationVars.ds=2; % 最大步数
CalibrationVars.numOfMaxSteps=50; % 最大步数
CalibrationVars.wOffset= 1;
CalibrationVars.wOffsetDer= 200; 
CalibrationVars.wOffsetSecDer= 1000; 
CalibrationVars.wOffsetThrDer= 50000; 
CalibrationVars.offsetMax=2;
CalibrationVars.offsetDerMax=2; 
CalibrationVars.offsetSecDerMax=10; 
CalibrationVars.offsetThrDerMax=100; 
CalibrationVars.FLAGS_lateral_derivative_bound_default=2;
CalibrationVars.MovingRoomFromCenterLane=1;
BasicInfo.widthOfVehicle=2;
BasicInfo.maxSteerAngle=8.20304748437;
BasicInfo.steerRatio=16;
BasicInfo.wheelBase=2.8448;
BasicInfo.maxSteerAngleRate=6.98131700798;

offsetTarget2CurrentLane=3.2;
sSequcence=[0,100,200];
lRefSequcence=[-1,-1,-1];
rSequcence=[2000,2000,2000];
offsetRight2CurrentLane=3.2;
offsetLeft2CurrentLane=3.2;
headingCurrent=0;

s_end=100;
l_0=0; %1.6
l_end=-1;
headingEnd=0;

tic
[lSequcence,exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,speed,sSequcence,lRefSequcence,rSequcence,headingCurrent,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
    CalibrationVars,BasicInfo);
toc
s = s_0+0:CalibrationVars.ds:CalibrationVars.numOfMaxSteps*CalibrationVars.ds;
plot(s,lSequcence);
% axis equal