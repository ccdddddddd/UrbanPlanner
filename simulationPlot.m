clc
clear
close all
% 添加第一个文件夹到路径
addpath('C:\Users\Chen Deng\Desktop\CodeManage\UrbanPlanner\MCTS');
% 添加第二个文件夹到路径
addpath('C:\Users\Chen Deng\Desktop\CodeManage\UrbanPlanner\pathPlan');
% 添加第三个文件夹到路径
addpath('C:\Users\Chen Deng\Desktop\CodeManage\UrbanPlanner\QP');
figure(1);
if isempty(gcf().Children)
    title('Planning trajectory')
    xlabel('x(meter)')
    ylabel('y(meter)')
else
    cla(gcf().Children)
end
hold on
%初始化
initparameter
%车道线
% waypoints = [0 0; 50 20; 100 0; 150 10 ;200,20;250,0;300,20];
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
%车道线入参
LaneLineInfo.currentLanePointsX = waypoints(:,1)';
LaneLineInfo.currentLanePointsY = waypoints(:,2)';
LaneLineInfo.leftLanePointsX = wayPointsLeft_global(:,1)';
LaneLineInfo.leftLanePointsY = wayPointsLeft_global(:,2)';
LaneLineInfo.rightLanePointsX = [];
LaneLineInfo.rightLanePointsY = [];
%实时入参
arraySize = [50, 1];
ObstacleInfo.obstacleNum = 0; %目标物数量
obstacles.x = zeros(51,1); %frenet 坐标
obstacles.y = zeros(51,1);%frenet 坐标
obstacles.theta = zeros(51,1);%在frenet坐标系横摆角
obstacles.speed = zeros(51,1);
obstacles.timestamp = zeros(51,1);
obstacles.length = 0; %目标物长度
obstacles.width = 0; %目标物宽度
ObstacleInfo.obstacles = repmat(obstacles, arraySize);

BasicsInfo.speed = 0; %车速
BasicsInfo.acce = 0;
BasicsInfo.v_max = 50/3.6; %期望车速
s = 0; %frenet 坐标
l = 0.5; %frenet 坐标
psi = 0;
vehFrenet = frenet2global(refPath,[0,0,0,0.5,0,0]);
BasicsInfo.x = vehFrenet(1); 
BasicsInfo.y = vehFrenet(2); 
BasicsInfo.theta = vehFrenet(3); 
BasicsInfo.SteerAngle = 0;
BasicsInfo.currentLaneIndex = 2; %当前车道序号
BasicsInfo.targetLaneIndex = 2; %目标车道序号
% BasicsInfo.d_veh2goal = 200; %与终点距离
% BasicsInfo.goalLaneIndex = 1; %终点车道
% BasicsInfo.trafficLightPhase = [30,-2,-2,-40];%信号灯相位
% BasicsInfo.d_veh2stopline = 200; %停止线距离
% BasicsInfo.d_veh2cross = 200; %与人行道距离
% BasicsInfo.d_veh2waitingArea = 200; %与待转区停止线距离
hold on
for i=1:1000
    i
    %环境车-----------------------------------------------------------------
    t_list = (0:0.1:5)';
    v = zeros(51,1)+5;
    s = i*0.1*v(1)+10+v.*t_list;
    car1_frenet = [s,v,zeros(51,1),zeros(51,1),zeros(51,1),zeros(51,1)]; 
    car1_global = frenet2global(refPath,car1_frenet);
    v = zeros(51,1)+7;
    s = i*0.1*v(1)+20+v.*t_list;
    car2_frenet = [s,v,zeros(51,1),zeros(51,1),zeros(51,1),zeros(51,1)]; 
    car2_global = frenet2global(refPathLeft,car2_frenet);
    ObstacleInfo.obstacleNum =2;
    ObstacleInfo.obstacles(1).x = car1_global(:,1); %frenet 坐标
    ObstacleInfo.obstacles(1).y = car1_global(:,2);%frenet 坐标
    ObstacleInfo.obstacles(1).theta = car1_global(:,3);%在frenet坐标系横摆角
    ObstacleInfo.obstacles(1).speed = car1_global(:,5);
    ObstacleInfo.obstacles(1).timestamp = zeros(51,1);
    ObstacleInfo.obstacles(1).length = 5; %目标物长度
    ObstacleInfo.obstacles(1).width = 2; %目标物宽度

    ObstacleInfo.obstacles(2).x = car2_global(:,1); %frenet 坐标
    ObstacleInfo.obstacles(2).y = car2_global(:,2);%frenet 坐标
    ObstacleInfo.obstacles(2).theta = car2_global(:,3);%在frenet坐标系横摆角
    ObstacleInfo.obstacles(2).speed = car2_global(:,5);
    ObstacleInfo.obstacles(2).timestamp = zeros(51,1);
    ObstacleInfo.obstacles(2).length = 5; %目标物长度
    ObstacleInfo.obstacles(2).width = 2; %目标物宽度
    %----------------------------------------------------------------------
    if i~=1
        % %参考线选择
        if i==50
            BasicsInfo.targetLaneIndex = 1; %目标车道序号
        end
        if i>51 && GlobVars.urbanPlanner.ischanginglanes == 0
            BasicsInfo.currentLaneIndex = 1;
            LaneLineInfo.currentLanePointsX = wayPointsLeft_global(:,1)';
            LaneLineInfo.currentLanePointsY = wayPointsLeft_global(:,2)';
            LaneLineInfo.leftLanePointsX = [];
            LaneLineInfo.leftLanePointsY = [];
        end

        %位置更新
        BasicsInfo.speed = traj.speed(2);
        BasicsInfo.acce =  traj.acce(2);
        BasicsInfo.x =  traj.x(2);
        BasicsInfo.y =  traj.y(2);
        BasicsInfo.theta = traj.theta(2);
    end
    %调用
    tic
    [traj,trajectoryType,GlobVars] = urbanplanner(BasicsInfo,ObstacleInfo,LaneLineInfo,GlobVars,Parameter,CalibrationVars);
    planningstatedisp(trajectoryType);
    toc

    %画图
    figure(1);
    if exist('p_v','var') ==1
        delete(p_v);
        delete(p_traj);
    end
    if exist('p_car','var') ==1
        delete(p_car);
        delete(p_cartraj);
    end
    p_v=plot(traj.x(1),traj.y(1),'co');
    p_traj = plot(traj.x,traj.y,'b.');
    for i_obstacle=1:ObstacleInfo.obstacleNum
        p_car(i_obstacle)=plot(ObstacleInfo.obstacles(i_obstacle).x(1),ObstacleInfo.obstacles(i_obstacle).y(1),'co');
        p_cartraj(i_obstacle) = plot(ObstacleInfo.obstacles(i_obstacle).x,ObstacleInfo.obstacles(i_obstacle).y,'y.');
    end
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