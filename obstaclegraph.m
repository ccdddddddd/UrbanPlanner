function obstacleMap=obstaclegraph(ObstaclesFrenetState,pathLine,offset)

% fprime = fnder(pathLine,1);%一阶导
obstacleNum = ObstaclesFrenetState.obstacleNum; %目标物数量
obstacles = ObstaclesFrenetState.obstacles;
obstacleMap = cell(1, 51);
if obstacleNum ~= 0
    len = length(obstacles(1).s);
    obstacleMap = cell(1, len);
    for i=1:obstacleNum
        for jt=1:len
            %         Obstacles(j).s(i)
            %         Obstacles(j).l(i)
            %         Obstacles(j).psi(i)
            vertices = calculateRectangleVertices(obstacles(i).s(jt), obstacles(i).l(jt), obstacles(i).psi(jt), obstacles(i).length, obstacles(i).width);
            [rR_s,~,rR_theta,rR_dis] = calculateProjectionPoint(pathLine,vertices(1,1),vertices(1,2));
            [rF_s,~,rF_theta,rF_dis] = calculateProjectionPoint(pathLine,vertices(2,1),vertices(2,2));
            [lF_s,~,lF_theta,lF_dis] = calculateProjectionPoint(pathLine,vertices(3,1),vertices(3,2));
            [lR_s,~,lR_theta,lR_dis] = calculateProjectionPoint(pathLine,vertices(4,1),vertices(4,2));
            if min([rR_dis,rF_dis,lF_dis,lR_dis]) < offset
                minS = min([rR_s,rF_s,lF_s,lR_s]);
                maxS = max([rR_s,rF_s,lF_s,lR_s]);
                thetaR = mean([rR_theta,rF_theta,lF_theta,lR_theta]);
                objects_ij = [minS,maxS,obstacles(i).speed(jt)*cosd(obstacles(i).psi(jt)-thetaR)];
                obstacleMap{1,jt} = [obstacleMap{1,jt};objects_ij];
                %             s_start = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), minS)+pathLine.breaks(1);
                %             s_end = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), maxS)+pathLine.breaks(1);
                %             if s_end> pathStart_s || s_start < pathEnd_s
                %                 objects_ij = [objects_ij;s_start,s_end,Obstacles(j).speed(i)];
                %             end
            end
        end
    end
end
%找投影点
function [x1,y1,theta,fval] = calculateProjectionPoint(splinedCurve,x0,y0)
    % 给定点的坐标
    point = [x0, y0];
    % 定义目标函数（垂直距离的平方）
    objectiveFunction = @(t) (point(2) - ppval(splinedCurve, t))^2+(point(1)-t)^2;
    % 最小化目标函数，得到最佳的参数值 t
    [t_optimal,fval] = fminbnd(objectiveFunction, min(splinedCurve.breaks), max(splinedCurve.breaks));
    % 计算投影点的坐标
    x1 = t_optimal;
    y1 = ppval(splinedCurve, t_optimal);
    dy1 = ppval(fnder(splinedCurve,1), t_optimal);
    theta = atand(dy1);
end
%计算车四个顶点坐标
function vertices = calculateRectangleVertices(x, y, theta, len, width)
    % 将夹角转换为弧度
    theta = deg2rad(theta);
    % 计算矩形的半宽度和半高度
    halflength = len / 2;
    halfwidth = width / 2;
    % 计算矩形的四个顶点相对于中心的坐标
    relativeVertices = [
        -halflength, -halfwidth;
        halflength, -halfwidth;
        halflength, halfwidth;
        -halflength, halfwidth;
    ];
    % 创建旋转矩阵
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    % 将矩形的四个顶点旋转并平移至中心位置
    rotatedVertices = (rotationMatrix * relativeVertices')';
    % 平移至中心坐标
    translatedVertices = rotatedVertices + [x, y];
    vertices = translatedVertices;
end
end
