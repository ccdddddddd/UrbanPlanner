function ObstaclesFrenetState = obstacle2frenetstate(refLine,ObstacleInfo)
%将Obstacle xy坐标转成frenet坐标系下坐标，psi为theta-thetaR
ObstaclesFrenetState.obstacleNum = ObstacleInfo.obstacleNum; %目标物数量
obstacles.s = zeros(51,1); %frenet 坐标
obstacles.l = zeros(51,1);%frenet 坐标
obstacles.psi = zeros(51,1);%在frenet坐标系横摆角
obstacles.speed = zeros(51,1);%车速
obstacles.length = 0; %目标物长度
obstacles.width = 0; %目标物宽度
% obstacles.projectionPointOnLine = zeros(51,1);
ObstaclesFrenetState.obstacles = repmat(obstacles, [ObstacleInfo.obstacleNum, 1]);
for index=1:ObstacleInfo.obstacleNum
[pathPoints,~] = closestPointsToSequence(refLine,[ObstacleInfo.obstacles(index).x,ObstacleInfo.obstacles(index).y],[0,refLine.PathLength]);
ObstaclesFrenetState.obstacles(index).s = pathPoints(:,6);
ObstaclesFrenetState.obstacles(index).l = sqrt((ObstacleInfo.obstacles(index).x-pathPoints(:,1)).^2+(ObstacleInfo.obstacles(index).y-pathPoints(:,2)).^2).*...
    sign((ObstacleInfo.obstacles(index).y-pathPoints(:,2)).*cos(pathPoints(:,3))-(ObstacleInfo.obstacles(index).x-pathPoints(:,1)).*sin(pathPoints(:,3)));
ObstaclesFrenetState.obstacles(index).psi = ObstacleInfo.obstacles(index).theta - pathPoints(:,3);
ObstaclesFrenetState.obstacles(index).speed = ObstacleInfo.obstacles(index).speed;
ObstaclesFrenetState.obstacles(index).length = ObstacleInfo.obstacles(index).length;
ObstaclesFrenetState.obstacles(index).width = ObstacleInfo.obstacles(index).width;
% ObstaclesFrenetState.obstacles(index).projectionPointOnLine = projectionPointOnLine;
end
end