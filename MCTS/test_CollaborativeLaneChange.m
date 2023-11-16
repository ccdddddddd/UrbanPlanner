clc
clear
close all

v_max=50/3.6;
numOfVehicles=int16(2);
numOfLanes=int16(3);
linkMapIds=zeros(5,5,'int16');
linkMapSs=zeros(5,5);
linkMapSpeeds=zeros(5,5);
linkMapLaneIndexs=zeros(1,25,'int16');
stateIds=zeros(1,5,'int16');
stateSpeeds=zeros(1,5);
stateLanes=zeros(1,5,'int16');
stateTargetSpeeds=zeros(1,5);
stateTargetLanes=zeros(1,5,'int16');
statePreDecision=zeros(1,5,'int16');
failVehicleIdList=zeros(1,10,'int16');
%标定量
CalibrationVars.numOfMaxSteps=int16(5);
CalibrationVars.accel=1.5;
CalibrationVars.decel=-1.5;
CalibrationVars.stepLength=2; % s
CalibrationVars.gamma=0.8; % TBD
CalibrationVars.numOfIteration=int16(4500);

CalibrationVars.a_min=-2;
CalibrationVars.a_max=1.5;
CalibrationVars.tou=1;%驾驶员响应时间

CalibrationVars.v_min_decel=3;
CalibrationVars.p_actionTime=0.5;

CalibrationVars.d_safe=10;
CalibrationVars.t_re=0.5;
CalibrationVars.w_lane=3.2;
CalibrationVars.w_veh=1.8;
CalibrationVars.epsilonSwitch=int16(5);

CalibrationVars.multipleOfLaneChange=10;
CalibrationVars.multipleOfNearGoalStateLane=10;
CalibrationVars.multipleOfNearGoalStateSpeed=1;
CalibrationVars.multipleOfKeepGoalState=5;
CalibrationVars.indexRewardOfAccelDecelSwitch=-1;
CalibrationVars.debugFlag=true;

standardActionTillState=zeros(5,2);
timeOfTest=1;
PercentageStandardActionTillState=0;
for caseIndex=101:1:101
    PercentageStandardActionTillState=0;
    PercentageStandardOptimalActive=0;
    
    if caseIndex<200
    switch caseIndex
        case 0
%             standardActionTillState=[2,5;1,5;4,5;5,5;5,5];
            standardActionTillState=[2,5;5,5;1,5;4,5;5,5];
            linkMapIds(1,:)=[11,100,12,0,0];
            linkMapSs(1,:)=[0,35,120,0,0];
            linkMapSpeeds(1,:)=[50/3.6,50/3.6,50/3.6,0,0];
            
            linkMapIds(2,:)=[21,101,22,0,0];
            linkMapSs(2,:)=[0,40,120,0,0];
            linkMapSpeeds(2,:)=[50/3.6,50/3.6,50/3.6,0,0];
            
            linkMapIds(3,:)=[31,0,0,0,0];
            linkMapSs(3,:)=[40,0,0,0,0];
            linkMapSpeeds(3,:)=[50/3.6,0,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 1;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 2;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 2;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;
        case 1
            standardActionTillState=[3,3;5,5;4,5;5,4;5,5]; % TBD
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[11,12,0,0,0];
            linkMapSs(1,:)=[30,80,0,0,0];
            linkMapSpeeds(1,:)=[50/3.6,50/3.6,0,0,0];
            
            linkMapIds(2,:)=[21,22,0,0,0];
            linkMapSs(2,:)=[0,80,0,0,0];
            linkMapSpeeds(2,:)=[50/3.6,50/3.6,0,0,0];
            
            linkMapIds(3,:)=[101,100,32,0,0];
            linkMapSs(3,:)=[20,40,75,0,0];
            linkMapSpeeds(3,:)=[50/3.6,50/3.6,0,0,0];
            failVehicleIdList(1)=32;
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 3;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 3;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 3;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 3;
        case 2
%             standardActionTillState=[5,3;3,5;5,5;4,5;5,4]; % TBD
            standardActionTillState=[2,3;3,5;1,4;4,5;5,5];
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[11,12,0,0,0];
            linkMapSs(1,:)=[20,65,0,0,0];
            linkMapSpeeds(1,:)=[50/3.6,50/3.6,0,0,0];
            
            linkMapIds(2,:)=[101,22,0,0,0];
            linkMapSs(2,:)=[45,85,0,0,0];
            linkMapSpeeds(2,:)=[50/3.6,50/3.6,0,0,0];
            
            linkMapIds(3,:)=[31,100,32,0,0];
            linkMapSs(3,:)=[15,40,100,0,0];
            linkMapSpeeds(3,:)=[50/3.6,50/3.6,0,0,0];
            failVehicleIdList(1)=32;
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 3;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 3;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 2;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;
        case 3
%             standardActionTillState=[2,4;2,5;3,2;5,5;1,5];  % TBD
            % standardActionTillState=[3,2;2,2;5,4;2,5;1,1]; standardActionTillState=[3,2;5,2;2,4;5,2;5,1];
%             standardActionTillState=[3,2;2,2;5,4;5,1;1,1];
            standardActionTillState=[2,4;2,2;3,5;1,5;1,1]; 
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,11,0,0,0];
            linkMapSs(1,:)=[20,90,0,0,0];
            linkMapSpeeds(1,:)=[50/3.6,0,0,0,0];
            
            linkMapIds(2,:)=[21,22,0,0,0];
            linkMapSs(2,:)=[70,85,0,0,0];
            linkMapSpeeds(2,:)=[50/3.6,50/3.6,0,0,0];
            
            linkMapIds(3,:)=[31,100,32,0,0];
            linkMapSs(3,:)=[10,20,90,0,0];
            linkMapSpeeds(3,:)=[50/3.6,50/3.6,0,0,0];
            
            failVehicleIdList(1)=11;
            failVehicleIdList(2)=32;
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 3;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 2;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;       
        case 4
            standardActionTillState=[5,3;5,2;5,5;4,1;5,5]; % TBD
            
            numOfVehicles=2;
   
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[100,0,0,0,0];
            linkMapSs(1,:)=[50,150,0,0,0];
            linkMapSpeeds(1,:)=[40/3.6,0,0,0,0];
            
            linkMapIds(2,:)=[0,0,0,0,0];
            linkMapSs(2,:)=[0,0,0,0,0];
            linkMapSpeeds(2,:)=[0,0,0,0,0];
            
            linkMapIds(3,:)=[102,101,0,0,0];
            linkMapSs(3,:)=[20,50,200,0,0];
            linkMapSpeeds(3,:)=[40/3.6,40/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 1;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 2;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 3;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;
            
            stateIds(3)=102;
            stateSpeeds(3) = 50/3.6;
            stateLanes(3)= 3;
            stateTargetSpeeds(3) = 50/3.6;
            stateTargetLanes(3) = 2;
        case 5
            standardActionTillState=[2,2;1,2;5,1;3,1;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,12,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,16,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;
        case 6
            standardActionTillState=[2,2;1,2;5,1;3,1;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,15,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,15,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;  
        case 7
            standardActionTillState=[2,2;1,2;5,1;3,1;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,11,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,16,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;  
        case 8
            standardActionTillState=[5,2;5,1;3,5;5,5;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,7,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,16,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 9
            standardActionTillState=[5,2;5,1;3,5;5,5;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,25,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,15,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 10
            standardActionTillState=[5,2;5,1;3,5;5,5;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,6,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,16,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 11
            standardActionTillState=[2,2;5,5;3,2;1,1;1,1]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,16,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,29,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 12
            standardActionTillState=[2,5;3,2;1,2;1,1;2,1]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,16,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,28,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 13
            standardActionTillState=[2,2;5,5;3,2;1,1;1,1]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[0,15,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,29,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1; 
        case 14
            standardActionTillState=[2,2;5,1;3,2;1,5;1,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,16,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,24,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;  
        case 15
            standardActionTillState=[2,2;2,2;3,1;5,1;1,2]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,16,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,23,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;   
        case 16
            standardActionTillState=[2,5;5,2;3,5;1,5;1,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[-5,15,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,24,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;
        case 17
            standardActionTillState=[2,5;5,5;3,5;1,5;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[3,23,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,16,0,0,0];
            linkMapSpeeds(2,:)=[54/3.6,54/3.6,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 54/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;
        case 18
            standardActionTillState=[2,5;5,5;3,5;1,5;5,5]; % TBD
            
            v_max=54/3.6;
            numOfVehicles=2;
            numOfLanes=2;
            
            linkMapIds=zeros(5,5);
            linkMapSpeeds=zeros(5,5);
            linkMapSs=zeros(5,5);
            
            linkMapIds(1,:)=[101,1,0,0,0];
            linkMapSs(1,:)=[3,24,0,0,0];
            linkMapSpeeds(1,:)=[54/3.6,54/3.6,0,0,0];
            
            linkMapIds(2,:)=[100,2,0,0,0];
            linkMapSs(2,:)=[0,35,0,0,0];
            linkMapSpeeds(2,:)=[13,13,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 13;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 54/3.6;
            stateTargetLanes(1) = 1;
            
            stateIds(2)=101;
            stateSpeeds(2) = 54/3.6;
            stateLanes(2)= 1;
            stateTargetSpeeds(2) = 54/3.6;
            stateTargetLanes(2) = 1;
        case 20
            standardActionTillState=[5,2;5,1;4,2;5,1;5,5];
            
            linkMapIds(1,:)=[11,100,12,0,0];
            linkMapSs(1,:)=[0,44,65,0,0];
            linkMapSpeeds(1,:)=[50/3.6,50/3.6,50/3.6,0,0];
            
            linkMapIds(2,:)=[21,101,22,0,0];
            linkMapSs(2,:)=[10,40,70,0,0];
            linkMapSpeeds(2,:)=[50/3.6,50/3.6,50/3.6,0,0];
            
            linkMapIds(3,:)=[31,0,0,0,0];
            linkMapSs(3,:)=[40,0,0,0,0];
            linkMapSpeeds(3,:)=[50/3.6,0,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 1;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 2;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 2;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;
        case 100
%             standardActionTillState=[2,5;1,5;4,5;5,5;5,5];
            standardActionTillState=[2,5;2,5;1,5;4,5;1,5];
            linkMapIds(1,:)=[100,0,0,0,0];
            linkMapSs(1,:)=[40,0,0,0,0];
            linkMapSpeeds(1,:)=[50/3.6,0,0,0,0];
            
            linkMapIds(2,:)=[101,0,0,0,0];
            linkMapSs(2,:)=[40,0,0,0,0];
            linkMapSpeeds(2,:)=[50/3.6,0,0,0,0];
            
            linkMapIds(3,:)=[31,0,0,0,0];
            linkMapSs(3,:)=[40,0,0,0,0];
            linkMapSpeeds(3,:)=[50/3.6,0,0,0,0];
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 1;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 2;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 2;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 2;
        case 101
%             standardActionTillState=[2,5;1,5;4,5;5,5;5,5];
            standardActionTillState=[2,5;2,5;1,5;4,5;1,5];
            linkMapIds(2,:)=[100,0,0,0,0];
            linkMapSs(2,:)=[40,0,0,0,0];
            linkMapSpeeds(2,:)=[50/3.6,0,0,0,0];
            
            linkMapIds(3,:)=[101,0,0,0,0];
            linkMapSs(3,:)=[40,0,0,0,0];
            linkMapSpeeds(3,:)=[50/3.6,0,0,0,0];
            
            
            stateIds(1)=100;
            stateSpeeds(1)= 50/3.6;
            stateLanes(1) = 2;
            stateTargetSpeeds(1) = 50/3.6;
            stateTargetLanes(1) = 3;
            
            stateIds(2)=101;
            stateSpeeds(2) = 50/3.6;
            stateLanes(2)= 3;
            stateTargetSpeeds(2) = 50/3.6;
            stateTargetLanes(2) = 3;
    end
    
    
    linkMapIds=[linkMapIds(1,:),linkMapIds(2,:),linkMapIds(3,:),linkMapIds(4,:),linkMapIds(5,:)];
    linkMapSpeeds=[linkMapSpeeds(1,:),linkMapSpeeds(2,:),linkMapSpeeds(3,:),linkMapSpeeds(4,:),linkMapSpeeds(5,:)];
    linkMapSs=[linkMapSs(1,:),linkMapSs(2,:),linkMapSs(3,:),linkMapSs(4,:),linkMapSs(5,:)];
    linkMapLaneIndexs=[zeros(1,5)+1,zeros(1,5)+2,zeros(1,5)+3,zeros(1,5)+4,zeros(1,5)+5];
    linkMapIds=int16(linkMapIds);
    linkMapLaneIndexs=int16(linkMapLaneIndexs);
    end
    

%     if epsilonSwitch==3
%         epsilon=(epsilon).^(1/4);
%     elseif epsilonSwitch==2
%         epsilon=sqrt(epsilon);
%     elseif epsilonSwitch==4
%         epsilon=0.1;
%     elseif epsilonSwitch==5
%         epsilon=0.7;
%     end

    tic
    for k=1:timeOfTest
        [actionTillState,optimalAction]=CollaborativeLaneChange(linkMapIds,linkMapSpeeds,linkMapSs,linkMapLaneIndexs,stateIds,stateSpeeds,stateLanes,stateTargetSpeeds,...
            stateTargetLanes,statePreDecision,failVehicleIdList,v_max,numOfVehicles,numOfLanes,CalibrationVars);
%         [actionTillState,optimalAction,node]=CollaborativeLaneChange_mex(linkMapIds,linkMapSpeeds,linkMapSs,linkMapLaneIndexs,stateIds,stateSpeeds,stateLanes,stateTargetSpeeds,...
%             stateTargetLanes,failVehicleIdList,v_max,numOfVehicles,numOfLanes,CalibrationVars);
        PercentageStandardActionTillState=PercentageStandardActionTillState+...,
            (isequal(actionTillState,standardActionTillState)-PercentageStandardActionTillState)/k;
        PercentageStandardOptimalActive=PercentageStandardOptimalActive+...,
            (isequal(optimalAction,standardActionTillState)-PercentageStandardOptimalActive)/k;
    end
    caseIndex % case序号
    PercentageStandardActionTillState  % 设置的标准解出现概率
    PercentageStandardOptimalActive 
    toc
    actionTillState
    optimalAction
    % t=toc/20
    %%  画图
%     nStar=selectionBest(node);
%     nSet=zeros(node(nStar).Generation+1,1);
%     for k=node(nStar).Generation:-1:0
%         nSet(k+1)=nStar;
%         nStar=node(nStar).parent;
%     end
%     for i=1:length(nSet)
%         plotScenes(node(nSet(i)).linkMap,node(1).state)
%         pause(0.8)
%     end

end
%backup
% for i=1:1:25
%     if node(35).children(i)~=0
%         node(35).children(i)
%         node(node(35).children(i)).actionTillState
%         node(node(35).children(i)).totalReward
%         
%     end
% end
% for i=1:1:25
%     if node(156).children(i)~=0
%         node(156).children(i)
%         node(node(156).children(i)).actionTillState
%     end
% end