clc
clear
close all
CalibrationVars.accel=2.5;
CalibrationVars.decel=-4;
CalibrationVars.jMax=10;
CalibrationVars.jMin=-10;
CalibrationVars.offsetMax=10; % 与MCTS生成轨迹的最大可接受偏差
CalibrationVars.numOfMaxSteps=50; %50; % 最大步数
CalibrationVars.wOffset=0.5; % 0.5; 对角线上元素的值为wOffset
CalibrationVars.wJerk=1-CalibrationVars.wOffset; % 0.5; % 对角线上元素的值为wJerk
testCase=0;
dt=0.1;

switch testCase
    case 0
        v_0=10;
        s_0=0;
        a_0=-1;
        v_maxVehicle=30;
        load("resultMCTS.mat")
        v_end=speedList(end);
        s_end=sList(end);
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        obstacleMapFuture=obstacleMap(2:51);
        for iInObatacleMap=1:1:50
            if ~isempty(obstacleMapFuture{iInObatacleMap})
                intervals = obstacleMapFuture{iInObatacleMap}(:,1:2);
                % 要检查的值
                value = sList(iInObatacleMap);
                % 查找小于value的不可行区域上界
                lowerBound = intervals(intervals(:,2) < value, 2);
                lowerBound = max(lowerBound);
                % 查找大于value的不可行区域上界
                upperBound = intervals(intervals(:,1) > value, 1);
                upperBound = min(upperBound);
                if ~isempty(lowerBound)
                    sMinSequenceMCTS(iInObatacleMap)=lowerBound;
                end
                if ~isempty(upperBound)
                    sMaxSequenceMCTS(iInObatacleMap)=upperBound;
                end
            end
        end
        sMCTS=sList'; % [1,2,3];
    case 1
        sMCTS=zeros(1,CalibrationVars.numOfMaxSteps); % [1,2,3];
        v_0=10;
        s_0=0;
        a_0=-999;
        v_end=-999;
        s_end=-999;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_maxVehicle=20;
    case 2
        v_0=10;
        sMCTS=v_0*dt:v_0*dt:v_0*dt*CalibrationVars.numOfMaxSteps; % [1,2,3];
        s_0=0;
        a_0=-999;
        v_end=-999;
        s_end=-999;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_maxVehicle=20;
    case 3
        v_0=10;
        sMCTS=v_0*dt:v_0*dt:v_0*dt*CalibrationVars.numOfMaxSteps; % [1,2,3];
        s_0=0;
        a_0=2;
        v_end=-999;
        s_end=-999;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_maxVehicle=20;
    case 4
        v_0=10;
        sMCTS=v_0*dt:v_0*dt:v_0*dt*CalibrationVars.numOfMaxSteps; % [1,2,3];
        s_0=0;
        a_0=2;
        v_end=0;
        s_end=25;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+999;
        v_maxVehicle=20;
    case 5
        v_0=10;
        sMCTS=v_0*dt:v_0*dt:v_0*dt*CalibrationVars.numOfMaxSteps; % [1,2,3];
        s_0=0;
        a_0=2;
        v_end=0;
        s_end=35;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+100;
        sMinSequenceMCTS(10:15)=10.8;
        sMinSequenceMCTS(30:35)=23.8;
        v_maxVehicle=20;
    case 6
        v_0=10;
        sMCTS=v_0*dt:v_0*dt:v_0*dt*CalibrationVars.numOfMaxSteps; % [1,2,3];
        s_0=0;
        a_0=2;
        v_end=-999;
        s_end=-999;
        sMinSequenceMCTS=zeros(1,CalibrationVars.numOfMaxSteps)+s_0;
        sMaxSequenceMCTS=sMinSequenceMCTS+30;
        sMinSequenceMCTS(10:15)=10.8;
        sMinSequenceMCTS(30:35)=23.8;
        v_maxVehicle=20;
end
[jOptSequence,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_maxVehicle,CalibrationVars);
exitflag;
switch testCase
    case 0
        % 加载.fig文件
        fig = openfig('resultMCTS.fig');
        hold on;
        %         plot(dt:dt:CalibrationVars.numOfMaxSteps*dt,sMaxSequenceMCTS);
        %         hold on;
        %         plot(dt:dt:CalibrationVars.numOfMaxSteps*dt,sMinSequenceMCTS);
        %         hold on;
        % 画图
        p1=plot(0:dt:CalibrationVars.numOfMaxSteps*dt,sOptSequence,'--','LineWidth',2);
        % 添加图例
        legend(p1,'optimal curve by QP');
    otherwise
        plot(dt:dt:CalibrationVars.numOfMaxSteps*dt,sMCTS);
        hold on;
        plot(dt:dt:CalibrationVars.numOfMaxSteps*dt,sMaxSequenceMCTS);
        hold on;
        plot(dt:dt:CalibrationVars.numOfMaxSteps*dt,sMinSequenceMCTS);
        hold on;
        plot(0:dt:CalibrationVars.numOfMaxSteps*dt,sOptSequence);
        hold on;
        plot(0:dt:CalibrationVars.numOfMaxSteps*dt,vOptSequence);
        hold on;
        plot(0:dt:CalibrationVars.numOfMaxSteps*dt,aOptSequence);
        hold on;
        plot(0:dt:(CalibrationVars.numOfMaxSteps-1)*dt,jOptSequence);
        legend('sMCTS','sMaxSequenceMCTS','sMinSequenceMCTS','sOptSequence', 'vOptSequence', 'aOptSequence','jOptSequence');
end