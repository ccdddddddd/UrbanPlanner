function obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,pathStart_s,pathEnd_s)
% if lineType==1
% % 使用样条插值
% pathLine = spline(x, y);
% 
% else
% 
% end
fprime = fnder(pathLine,1);%一阶导
objectsNum = ObjectsInfo.objectsNum; %目标物数量
objects = ObjectsInfo.objects;
len = length(objects(1).s);
obstacleMap = cell(1, len);
for i=1:len
    objects_ij = [];
    for j=1:objectsNum
        objects(j).s(i)
        objects(j).l(i)
        objects(j).psi(i)
        vertices = calculateRectangleVertices(objects(j).s(i), objects(j).l(i), objects(j).psi(i), objects(j).length, objects(j).width);
        [rR_s,~,rR_dis] = calculateProjectionPoint(pathLine,vertices(1,1),vertices(1,2));
        [rF_s,~,rF_dis] = calculateProjectionPoint(pathLine,vertices(2,1),vertices(2,2));
        [lF_s,~,lF_dis] = calculateProjectionPoint(pathLine,vertices(3,1),vertices(3,2));
        [lR_s,~,lR_dis] = calculateProjectionPoint(pathLine,vertices(4,1),vertices(4,2));
        if min(rR_dis,rF_dis,lF_dis,lR_dis) < offset
            minS = min(rR_s,rF_s,lF_s,lR_s);
            maxS = max(rR_s,rF_s,lF_s,lR_s);
            s_start = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), minS)+pathLine.breaks(1);
            s_end = integral(@(x) sqrt(1 + ppval(fprime,x)), pathLine.breaks(1), maxS)+pathLine.breaks(1);
            if s_end> pathStart_s || s_start < pathEnd_s
                objects_ij = [objects_ij;s_start,s_end,objects(j).speed(i)];
            end
        end
    end
    obstacleMap{1,i}=objects_ij;
end
%找投影点
function [x1,y1,fval] = calculateProjectionPoint(splinedCurve,x0,y0)
    % 给定点的坐标
    point = [x0, y0];
    % 定义目标函数（垂直距离的平方）
    objectiveFunction = @(t) (point(2) - ppval(splinedCurve, t))^2+(point(1)-t)^2;
    % 最小化目标函数，得到最佳的参数值 t
    [t_optimal,fval] = fminbnd(objectiveFunction, min(splinedCurve.breaks), max(splinedCurve.breaks));
    % 计算投影点的坐标
    projectionPoint = ppval(splinedCurve, t_optimal);
    x1 = t_optimal;
    y1 = projectionPoint;
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
