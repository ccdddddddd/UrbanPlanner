
clear
close all
% openfig('untitled.fig')
s_0=0;
CalibrationVars.ds=1; % 最大步数
CalibrationVars.numOfMaxSteps=round(100/CalibrationVars.ds); % 最大步数
CalibrationVars.wOffset= 1;
CalibrationVars.wOffsetDer= 50; 
CalibrationVars.wOffsetSecDer= 1000; 
CalibrationVars.wOffsetThrDer= 50000; 
% lane_change_path_config
% CalibrationVars.wOffset= 1;
% CalibrationVars.wOffsetDer= 5; 
% CalibrationVars.wOffsetSecDer= 800; 
% CalibrationVars.wOffsetThrDer= 30000; 
% CalibrationVars.offsetMax=2;
% CalibrationVars.offsetDerMax=2; 
% CalibrationVars.offsetSecDerMax=10; 
% CalibrationVars.offsetThrDerMax=100; 
CalibrationVars.FLAGS_lateral_derivative_bound_default=2;
CalibrationVars.MovingRoomFromCenterLane=1;
CalibrationVars.steerAngleRateRatioMax=2;
CalibrationVars.steerAngleRateRatioMin=1.5;
CalibrationVars.stepByOffsetBound=0.5;
BasicInfo.widthOfVehicle=2;
BasicInfo.maxSteerAngle=8.20304748437;
BasicInfo.steerRatio=16;
BasicInfo.wheelBase=2.8448;
BasicInfo.maxSteerAngleRate=6.98131700798;
sSequcence=[0,100,200];
rSequcence=[2000,2000,2000];
offsetRight2CurrentLane=3.2;
offsetLeft2CurrentLane=3.2;
testcase=3;
switch testcase
    case 0
        speed=2;
        headingCurrent=45;
        s_end=100;
        l_0=0; %1.6
        l_end=0;
        headingEnd=0;
        lRefSequcence=[0,0,0];
    case 1
        speed=10;
        headingCurrent=0;
        s_end=100;
        l_0=0; %1.6
        l_end=-0.8;
        headingEnd=0;
        lRefSequcence=[-0.8,-0.8,-0.8];
    case 2
        speed=20;
        headingCurrent=10;
        s_end=100;
        l_0=0.7; %1.6
        l_end=0;
        headingEnd=0;
        lRefSequcence=[0,0,0];
    case 3
        speed=5;
        headingCurrent=60;
        s_end=100;
        l_0=0; %1.6
        l_end=0;
        headingEnd=0;
        lRefSequcence=[0,0,0];
    otherwise
end
tic
[lSequcence,exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,speed,sSequcence,lRefSequcence,rSequcence,headingCurrent,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
    CalibrationVars,BasicInfo);
toc
s = s_0+0:CalibrationVars.ds:CalibrationVars.numOfMaxSteps*CalibrationVars.ds;
if exitflag
    figure('Position', [0, 100, 900, 100+400]) % 创建一个指定大小的图形窗口
    p0=plot(s,lSequcence);
    title('S-l Curve of Replan Path');
    xlabel('s(m)') % 设置 x 轴标签
    ylabel('l(m)') % 设置 y 轴标签
    xlim([0, 50]) % 设置 x 轴的范围为 0 到 100
    % ylim([0, 50]) % 设置 x 轴的范围为 0 到 100
    y = -offsetRight2CurrentLane/2;
    x = xlim; % 获取当前坐标轴的y范围
    p1=line(x,[y y], 'Color', 'r','LineStyle','--'); % 画蓝色直线
    y = offsetLeft2CurrentLane/2;
    x = xlim; % 获取当前坐标轴的y范围
    p2=line(x,[y y], 'Color', 'b','LineStyle','--'); % 画蓝色直线
    y = 0;
    x = xlim; % 获取当前坐标轴的y范围
    p3=line(x,[y y], 'Color', 'g','LineStyle','--'); % 画蓝色直线
    y = lRefSequcence(1);
    x = xlim; % 获取当前坐标轴的y范围
    p4=line(x,[y y], 'Color', 'c','LineStyle','--'); % 画蓝色直线
    if testcase~=1
        legend([p0,p1,p2,p3],...,
        'path of ego car', 'lane right sideline', 'lane left sideline', 'lane centerline', ...,
        'Location','northeast','FontSize',10);
    else
        legend([p0,p1,p2,p3,p4],...,
        'path of ego car', 'lane right sideline', 'lane left sideline', 'lane centerline', 'position to pull over', ...,
        'Location','northeast','FontSize',10);
    end
end
% axis equal