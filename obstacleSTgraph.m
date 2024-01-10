function obstacleMap=obstacleSTgraph(ObstacleInfo,refPath,offset)

objectsNum = ObstacleInfo.obstacleNum; %目标物数量
obstacles = ObstacleInfo.obstacles;
len = length(obstacles(1).x);
obstacleMap = cell(1, len);
initWindow = [0,refPath.PathLength];
for i=1:objectsNum
    vertices = calculateRectangleVertices(obstacles(i).x, obstacles(i).y, obstacles(i).theta, obstacles(i).length, obstacles(i).width);
    [pathPoints_rr,inWindow_rr] = closestPointsToSequence(refPath,vertices(:,1:2),initWindow);
    [pathPoints_rf,inWindow_rf] = closestPointsToSequence(refPath,vertices(:,3:4),initWindow);
    [pathPoints_lf,inWindow_lf] = closestPointsToSequence(refPath,vertices(:,5:6),initWindow);
    [pathPoints_lr,inWindow_lr] = closestPointsToSequence(refPath,vertices(:,7:8),initWindow);
    inWindow = inWindow_rr|inWindow_rf|inWindow_lf|inWindow_lr;
    dis2path_rr = sqrt((vertices(:,1)-pathPoints_rr(:,1)).^2+(vertices(:,2)-pathPoints_rr(:,2)).^2);
    dis2path_rf = sqrt((vertices(:,3)-pathPoints_rf(:,1)).^2+(vertices(:,4)-pathPoints_rf(:,2)).^2);
    dis2path_lf = sqrt((vertices(:,5)-pathPoints_lf(:,1)).^2+(vertices(:,6)-pathPoints_lf(:,2)).^2);
    dis2path_lr = sqrt((vertices(:,7)-pathPoints_lr(:,1)).^2+(vertices(:,8)-pathPoints_lr(:,2)).^2);
    inWindow = (dis2path_rr<offset | dis2path_rf<offset | dis2path_lf<offset | dis2path_lr<offset) & inWindow;
    for j=1:len
        if inWindow(j) == 1
            s_start = min([pathPoints_rr(j,6),pathPoints_rf(j,6),pathPoints_lf(j,6),pathPoints_lr(j,6)]);
            s_end = max([pathPoints_rr(j,6),pathPoints_rf(j,6),pathPoints_lf(j,6),pathPoints_lr(j,6)]);
            theta_r = mean([pathPoints_rr(j,3),pathPoints_rf(j,3),pathPoints_lf(j,3),pathPoints_lr(j,3)]);
            s_dot = obstacles(i).speed(j)*cosd(obstacles(i).theta(j)-theta_r);
            obstacleMap{1,j} = [obstacleMap{1,j};[s_start,s_end,s_dot]];
        end
    end
end
%计算车四个顶点坐标
    function vertices = calculateRectangleVertices(x, y, thetas, len, width)
        numOfPoints= length(x);
        MultiVerticesRelative = zeros(numOfPoints,8);
        for i_points=1:numOfPoints
%             theta = deg2rad(thetas(i_points));
            theta = thetas(i_points);
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
            rotatedVertices = (rotationMatrix * relativeVertices')';%[rr;rf;lf;lr]
            %转置 [xrr,yrr;xry,yrf;xlf,ylf;xlr,ylr]to[xrr,yrr,xry,yrf,xlf,ylf,xlr,ylr]
            VerticesRelative = rotatedVertices';
            MultiVerticesRelative(i_points,:) = VerticesRelative(:)';
        end
        % 平移至中心坐标
        vertices = MultiVerticesRelative+[x, y, x, y, x, y, x, y];%[xrr,yrr,xry,yrf,xlf,ylf,xlr,ylr]
    end
end