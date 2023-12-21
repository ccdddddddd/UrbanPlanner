function [pathPara,s_end_opt,laneChangeDec] = laneChangePathPlan(s_0,l_0,v_0,turningRadius,offsetTarget2CurrentLane,sSequcence,offsetTarget2CurrentLaneSequcence...,
    ,headingTargetLaneSequcence,headingCurrent,obstacleMap,obstacleMapTargetLane,CalibrationVars,BasicInfo,plotFlag,rectangles,rectanglesTargetLane) % ???如何在当前车道坐标系下表示目标车道车道中心线
% laneChangeDec为换道可行性，pathPara为三次多项式系数
% v_0=20;turningRadius=5;w=0.5;tmax=6;amax=1.5;amin=-2;vmax=20;vmin=6;
%% 标定量和初始化赋值
alateralmax=CalibrationVars.a_lateral; % =3;
w=CalibrationVars.wComfort2EfficiencyLaneChange; % =0.5; Jchg=w*(aLst/aMax).^2+(1-w)*(xEnd/xEndMax).^2
tmax=CalibrationVars.tMaxLaneChange; % =6;
amax=CalibrationVars.aMaxLaneChange; % 1.5;
amin=CalibrationVars.aMinLaneChange; % -2;
vmax=CalibrationVars.vMaxLaneChange; % 20;
vmin=CalibrationVars.vMinLaneChange; % 6;
decel=CalibrationVars.decel; % =-4;
speedGap=CalibrationVars.speedGap; % =2
linspaceNum=int16(CalibrationVars.linspaceNum); % =50
widthOfVehicle=BasicInfo.widthOfVehicle;
% offsetTarget2CurrentLane= 3.2; % 通过车道宽度计算得到

%% 备选换道结束时刻车速集合 → 换道路径集合 → 满足动力学限制换道路径集合
% v0+amax*tmax
% min(vmax,v0+amax*tmax)
xendmax=max(sqrt(turningRadius.^2-(turningRadius-abs(offsetTarget2CurrentLane-l_0)).^2),sqrt(((v_0+min(vmax,v_0+amax*tmax))/2*tmax).^2-abs(offsetTarget2CurrentLane-l_0).^2));
vendmin=max(vmin,v_0+amin*tmax);
vendmax=min(vmax,v_0+amax*tmax);
interal=(vendmax-vendmin)/round((vendmax-vendmin)/speedGap);
vendList=vendmin:interal:vendmax;
tendList=zeros(1,length(vendList));
xendList=tendList;
accerlMeanList=tendList;
costList=tendList;
coefficientsList=zeros(4,length(vendList));
departureFromCurrentLaneTimeList=tendList-1;
instrusionIntoTargetLaneTimeList=tendList-1;
for i=1:1:length(vendList)
    % if i==7
    %     i
    % end
    minDistanceInSList=CalibrationVars.d_max;
    %% 计算xend
    vend=vendList(i);
    xend=(4*w*(6*vend^2*(offsetTarget2CurrentLane-l_0)/alateralmax)^2/((1-w)/xendmax))^0.2;
    %% 插值计算xend对应的yend
    % 找到xend+s_0所在的区间
    index = find(sSequcence <= xend+s_0, 1, 'last');
    % 线性插值
    if ~isempty(index) && index < numel(sSequcence)
        s1 = sSequcence(index);
        s2 = sSequcence(index + 1);
        y1 = offsetTarget2CurrentLaneSequcence(index);
        y2 = offsetTarget2CurrentLaneSequcence(index + 1);
        yend = y1 + (y2 - y1) * (xend+s_0 - s1) / (s2 - s1);
        heading1 = headingTargetLaneSequcence(index);
        heading2 = headingTargetLaneSequcence(index + 1);
        gradientTarget = tand(heading1 + (heading2 - heading1) * (xend+s_0 - s1) / (s2 - s1));
    else
        fprintf('error:s坐标超出范围\n');
        yend=offsetTarget2CurrentLane;
        gradientTarget = 0;
    end
    %% 计算路径长度，借助匀加速运动模型计算换道用时及平均加速度，平均加速度超过加速度极限则路径不满足动力学限制
    % 已知点和斜率
    gradientCurrent = tand(headingCurrent); % 在点（s_0，0）处的斜率
    % gradientTarget=1; % 在点（s_0，0）处的斜率
    % 构建方程组矩阵
    A = [s_0^3, s_0^2, s_0, 1;
        3*s_0^2, 2*s_0, 1, 0;
        (s_0+xend)^3, (s_0+xend)^2, (s_0+xend), 1;
        3*(s_0+xend)^2, 2*(s_0+xend), 1, 0];
    b = [l_0; gradientCurrent; yend; gradientTarget];
    % 求解方程组
    coefficients = A \ b;
    coefficientsList(:,i)=coefficients;
    % 定义曲线长度函数
    % coefficients
    % s_0 + xend
    % s_0
    % curve_length = @(s) ;
    % 定义曲线长度积分函数
    % integrand = @(s) integral(curve_length, s_0, s);
    % 计算横坐标从s_0到s_0+xend的曲线长度
    % length = integrand(s_0 + xend) - integrand(s_0);
    % syms s;
    curve_length=@(s)sqrt(1 + (3*coefficients(1)*s.^2 + 2*coefficients(2)*s + coefficients(3)).^2);
    % lengthS = integral(curve_length, s_0, s_0 + xend);
    lengthS = trapz(linspace(s_0,s_0 + xend,linspaceNum),curve_length(linspace(s_0,s_0 + xend,linspaceNum)));
    tend=lengthS/((v_0+vend)/2); % tend=sqrt(xend^2+yend^2)/((v0+vend)/2);
    accerlMean=(vend-v_0)/tend;
    accerlMeanList(i)=accerlMean;
    tendList(i)=tend;
    xendList(i)=xend;
    if ~(accerlMean>amin && accerlMean<amax)
        costList(i)=-1;
    end
    %% 满足动力学限制换道路径集合 → 无碰撞换道路径集合
    dt=0.1;
    %% 检测换道结束状态下的TTC
    if costList(i)>-1
        if round(tend/dt)+1<=length(obstacleMapTargetLane)
            tendTTC=tend;
            value = xendList(i);
            vendTTC=vend;
        else
            tendTTC=(length(obstacleMapTargetLane)-1)*dt;
            displacementTTC = v_0 * tendTTC + 0.5 * accerlMean * tendTTC.^2; % 位移数组
            fun_x=@(x)trapz(linspace(s_0,x,linspaceNum),curve_length(linspace(s_0,x,linspaceNum)));
            [value,~,~] = fzero(@(x)fun_x(x)-displacementTTC,[s_0 s_0+xendList(i)]);
            vendTTC=v_0+accerlMean * tendTTC;
        end
        if ~isempty(obstacleMapTargetLane{round(tendTTC/dt)+1})
            intervals = obstacleMapTargetLane{round(tendTTC/dt)+1}(:,1:2);
            cipvDistance = CalibrationVars.d_max;
            cipvSpeed=0;
            for iterInIntervals = 1:size(intervals, 1)
                if value < intervals(iterInIntervals, 1)
                    frontDistance = intervals(iterInIntervals, 1) - value;
                    frontSpeed=obstacleMapTargetLane{round(tendTTC/dt)+1}(iterInIntervals,3);
                else
                    frontDistance=CalibrationVars.d_max;
                    frontSpeed=0;
                end
                % 保存与前车的最小距离
                if frontDistance < cipvDistance
                    cipvDistance = frontDistance;
                    cipvSpeed=frontSpeed;
                end
            end
            % 判断值是否不满足跟车安全距离
            if cipvDistance<CalibrationVars.d_max && ...,
                    cipvDistance<CalibrationVars.d_safe+vendTTC*CalibrationVars.t_re+max(0,(-vendTTC.^2+cipvSpeed.^2)/(2*decel))% 存在前车 && 不满足跟车安全距离
                costList(i)=-2;
            end
        end
    end
    %% 借助匀加速运动模型检测碰撞
    if costList(i)>-1
        %% 检验当前车道上的碰撞（t = checkTimeGap:checkTimeGap:instrusionIntoTargetLaneTime;）
        %         % 计算曲线1的坐标
        %         x = [sSequcence(1),sSequcence(end)];
        %         y = [CalibrationVars.MovingRoomFromCenterLane+0.5*widthOfVehicle,CalibrationVars.MovingRoomFromCenterLane+0.5*widthOfVehicle]*sign(offsetTarget2CurrentLane);
        %         % 计算曲线2的坐标
        %         x2 = linspace(s_0,sSequcence(end),25); % 参数化曲线2
        %         y2 = polyval(coefficients, x2);
        %         % 计算交点坐标
        %         [xi, ~] = polyxpoly(x, y, x2, y2);
        % 将三次多项式曲线的方程与 y = y1 的方程联立
        equation = @(x) polyval(coefficients, x) - (CalibrationVars.MovingRoomFromCenterLane+0.5*widthOfVehicle)*sign(offsetTarget2CurrentLane);
        % 使用 fzero 函数求解方程的根（交点的 x 坐标）
        xi = fzero(equation, [s_0,s_0 + xend]);
        if ~isempty(xi)
            departureFromCurrentLaneS=xi(1);
            % departureFromCurrentLaneL=yi(1);
            % 计算曲线长度
            % syms xx;
            % curveLength = int(sqrt(1 + diff(poly2sym(coefficients), xx)^2), s_0, departureFromCurrentLaneS);
            % departureFromCurrentLaneLength = integral(curve_length, s_0, departureFromCurrentLaneS);
            departureFromCurrentLaneLength = trapz(linspace(s_0,departureFromCurrentLaneS,linspaceNum),curve_length(linspace(s_0,departureFromCurrentLaneS,linspaceNum)));

            % 计算行驶时间
            if accerlMean == 0
                departureFromCurrentLaneTime = departureFromCurrentLaneLength / v_0;
            else
                departureFromCurrentLaneTime = (-v_0 + sqrt(v_0^2 + 2 * accerlMean * departureFromCurrentLaneLength)) / accerlMean;
            end
            departureFromCurrentLaneTimeList(i)=departureFromCurrentLaneTime;
            % 计算各个时刻下的位移
            checkTimeGap = CalibrationVars.checkTimeGap; % 0.2; % 时间间隔（秒）
            t1 = checkTimeGap:checkTimeGap:min(5,round(departureFromCurrentLaneTime/dt)*dt); % 时间数组
            xCoords1=zeros(1,length(t1));
            % yCoords=xCoords;
            displacement=zeros(1,length(t1));
            for iterInDisplacement=1:1:length(t1) % 时间数组
                if ~isempty(obstacleMap{round(t1(iterInDisplacement)/dt)+1})
                    displacement(iterInDisplacement) = v_0 * t1(iterInDisplacement) + 0.5 * accerlMean * t1(iterInDisplacement).^2; % 位移数组
                    % 计算xCoords
                    % fun_a = @(x)sqrt(1+((3*coefficients(1)*x.^2+4*coefficients(2)*x.^3+5*coefficients(3)*x.^4)).^2);
                    % curve_length=@(s)sqrt(1 + (3*coefficients(1)*s.^2 + 2*coefficients(2)*s + coefficients(3)).^2);
                    fun_x=@(x)trapz(linspace(s_0,x,linspaceNum),curve_length(linspace(s_0,x,linspaceNum)));
                    [xCoords1(iterInDisplacement),~,~] = fzero(@(x)fun_x(x)-displacement(iterInDisplacement),[s_0 s_0+xend]);       
                    % syms xxx;
                    % f = poly2sym(coefficients, xxx);
                    % df = diff(f);
                    % integrand = sqrt(1 + df^2);
                    % xCoords(iterInDisplacement) = double(vpasolve(int(integrand, s_0, xxx) == displacement(iterInDisplacement), xxx));
                    % 计算yCoords
                    % yCoords(iterInDisplacement)  = polyval(coefficients, xCoords(iterInDisplacement));
                    % end
                    % safeFlag=1;
                    % for iterInDisplacement=1:1:length(t) % 时间数组
                    intervals = obstacleMap{round(t1(iterInDisplacement)/dt)+1}(:,1:2);
                    % 要检查的值
                    value = xCoords1(iterInDisplacement);
                    % 计算到每个区间的距离
                    minDistance = CalibrationVars.d_max;
                    for iterInIntervals = 1:size(intervals, 1)
                        if value < intervals(iterInIntervals, 1)
                            distance = intervals(iterInIntervals, 1) - value;
                        elseif value > intervals(iterInIntervals, 2)
                            distance = value - intervals(iterInIntervals, 2);
                        else
                            % 如果该值在区间内，距离为0
                            distance = 0;
                        end
                        % 保存最小距离
                        if distance < minDistance
                            minDistance = distance;
                        end
                    end
                    % 判断值是否在不可用区间内
                    if minDistance == 0
                        % safeFlag=0;
                        costList(i)=-3;
                        break;
                    else
                        if minDistance<minDistanceInSList
                            minDistanceInSList=minDistance;
                        end
                    end
                end
            end
            %         else
            %             costList(i)=-3;
            % departureFromCurrentLaneL=-999;
        end
    end
    if costList(i)>-1
        %% 检验目标车道上的碰撞（t = departureFromCurrentLaneTime:checkTimeGap:min(tend,5);）
        % 计算曲线1的坐标
        x = sSequcence;
        y = offsetTarget2CurrentLaneSequcence-(CalibrationVars.MovingRoomFromCenterLane+0.5*widthOfVehicle)*sign(offsetTarget2CurrentLane);
        %         % 计算曲线2的坐标
        %         x2 = linspace(s_0,sSequcence(end),int16(CalibrationVars.linspaceNumCrossCal)); % 参数化曲线2
        %         y2 = polyval(coefficients, x2);
        % 计算交点坐标
        % 找到s_0所在的区间
        index = find(x <= s_0, 1, 'last');
        % 线性插值
        if ~isempty(index) && index < numel(x)
            s1 = x(index);
            s2 = x(index + 1);
            l1 = y(index);
            l2 = y(index + 1);
            y4s_0 = l1 + (l2 - l1) * (s_0 - s1) / (s2 - s1);
        else
            fprintf('error:s坐标超出范围\n');
            y4s_0=offsetTarget2CurrentLane-(CalibrationVars.MovingRoomFromCenterLane+0.5*widthOfVehicle)*sign(offsetTarget2CurrentLane);
        end
        if l_0>=y4s_0 % polyval(coefficients, s_0)>=y4s_0
            xi=s_0;
        else
            equation = @(xUnKnown) polyval(coefficients, xUnKnown) - (interp1(x, y, xUnKnown));
            % 使用 fzero 函数求解方程的根（交点的 x 坐标）
            xi = fzero(equation, [s_0,s_0 + xend]);
            % [xi, ~] = polyxpoly(x, y, x2, y2);
        end
        %         xi
        %         % 定义三次多项式曲线的方程
        %         polynomial_equation = @(t) polyval(coefficients, t);
        %         % 定义曲线的方程
        %         curve_equation = @(t) interp1(x, y, t);
        %         % 求解方程组，得到交点的参数值
        %         % options = optimoptions('fsolve', 'Display', 'off');
        %         xi = fsolve(@(t) polynomial_equation(t) - curve_equation(t), 0.5,optimset('Display','off'));
        if ~isempty(xi)
            instrusionIntoTargetLaneS=xi(1);
            % instrusionIntoTargetLaneL=yi(1);
            % 计算曲线长度
            % syms xx;
            % curveLength = int(sqrt(1 + diff(poly2sym(coefficients), xx)^2), s_0, instrusionIntoTargetLaneS);
            % instrusionIntoTargetLaneLength = double(curveLength);
            % instrusionIntoTargetLaneLength = integral(curve_length, s_0, instrusionIntoTargetLaneS);
            instrusionIntoTargetLaneLength = trapz(linspace(s_0,instrusionIntoTargetLaneS,linspaceNum),curve_length(linspace(s_0,instrusionIntoTargetLaneS,linspaceNum)));
            % 计算行驶时间
            if accerlMean == 0
                instrusionIntoTargetLaneTime = instrusionIntoTargetLaneLength / v_0;
            else
                instrusionIntoTargetLaneTime = (-v_0 + sqrt(v_0^2 + 2 * accerlMean * instrusionIntoTargetLaneLength)) / accerlMean;
            end
            instrusionIntoTargetLaneTimeList(i)=instrusionIntoTargetLaneTime;
            % 计算各个时刻下的位移
            t2 = dt*round(instrusionIntoTargetLaneTime/dt):checkTimeGap:min(tend,5); % 时间数组
            xCoords2=zeros(1,length(t2));
            % yCoords=xCoords;
            displacement=zeros(1,length(t2));
            for iterInDisplacement=1:1:length(t2) % 时间数组
                if ~isempty(obstacleMapTargetLane{round(t2(iterInDisplacement)/dt)+1})
                    displacement(iterInDisplacement) =  v_0 * t2(iterInDisplacement) + 0.5 * accerlMean * t2(iterInDisplacement).^2; % 位移数组
                    % 计算xCoords
                    fun_x=@(x)trapz(linspace(s_0,x,linspaceNum),curve_length(linspace(s_0,x,linspaceNum)));
                    [xCoords2(iterInDisplacement),~,~] = fzero(@(x)fun_x(x)-displacement(iterInDisplacement),[s_0 s_0+xend]);       
                    % syms xxx;
                    % f = poly2sym(coefficients, xxx);
                    % df = diff(f);
                    % integrand = sqrt(1 + df^2);
                    % xCoords(iterInDisplacement) = double(vpasolve(int(integrand, s_0, xxx) == displacement(iterInDisplacement), xxx));
                    % 计算yCoords
                    % yCoords(iterInDisplacement)  = polyval(coefficients, xCoords(iterInDisplacement));
                    %             end
                    %             for iterInDisplacement=1:1:length(t) % 时间数组
                    intervals = obstacleMapTargetLane{round(t2(iterInDisplacement)/dt)+1}(:,1:2);
                    % 要检查的值
                    value = xCoords2(iterInDisplacement);
                    % 计算到每个区间的距离
                    minDistance = CalibrationVars.d_max;
                    for iterInIntervals = 1:size(intervals, 1)
                        if value < intervals(iterInIntervals, 1)
                            distance = intervals(iterInIntervals, 1) - value;
                        elseif value > intervals(iterInIntervals, 2)
                            distance = value - intervals(iterInIntervals, 2);
                        else
                            % 如果该值在区间内，距离为0
                            distance = 0;
                        end
                        % 保存最小距离
                        if distance < minDistance
                            minDistance = distance;
                        end
                    end
                    % 判断值是否在不可用区间内
                    if minDistance == 0
                        % safeFlag=0;
                        costList(i)=-4;
                        break;
                    else
                        if minDistance<minDistanceInSList
                            minDistanceInSList=minDistance;
                        end
                    end
                end
            end
            %         else
            %             costList(i)=-4;
            %             % instrusionIntoTargetLaneL=-999;
        end
    end
    %% 代价计算
    if costList(i)>-1
        % 从 coefficients 数组中获取多项式系数
        a3 = coefficients(1);
        a2 = coefficients(2);
        a1 = coefficients(3);
        % a0 = coefficients(4);
        % 计算多项式的导数
        p1 = [3*a3, 2*a2, a1];  % 一次导数
        p2 = [2*p1(1), p1(2)];  % 二次导数
        % 定义车辆行驶过程中的时间数组
        t = linspace(0+dt, tend-dt, int16(CalibrationVars.linspaceNumALateralCal));
        xCoords=zeros(1,length(t));
        displacement=zeros(1,length(t));
        % 计算车辆行驶过程中的速度
        v = v_0 + accerlMean*t;  % 速度
        %         % 计算车辆行驶过程中的位置
        %         xCoords = s_0 + v_0*t + 0.5*accerlMean*t.^2;  % 位置
        %         % 计算车辆行驶过程中的曲率半径
        %         r = abs((1 + (p1(1)*xCoords.^2 + p1(2)*xCoords+p1(3)).^2).^1.5) ./ (eps+abs(p2(1)*xCoords + p2(2)));
        %         % 定义车辆行驶过程中的时间数组
        %         t = [t1,t2];
        %         % 计算车辆行驶过程中的位置和速度
        %         xCoords = [xCoords1,xCoords2];  % 位置
        %         v = v_0 + accerlMean*t;  % 速度
        %         % 计算车辆行驶过程中的曲率半径
        %         r = abs((1 + (p1(1)*xCoords.^2 + p1(2)*xCoords+p1(3)).^2).^1.5) ./ (eps+abs(p2(1)*xCoords + p2(2)));
        for iterInDisplacement=1:1:length(t) % 时间数组
            displacement(iterInDisplacement) = v_0 * t(iterInDisplacement) + 0.5 * accerlMean * t(iterInDisplacement).^2; % 位移数组
            % 计算xCoords
            % fun_a = @(x)sqrt(1+((3*coefficients(1)*x.^2+4*coefficients(2)*x.^3+5*coefficients(3)*x.^4)).^2);
            % curve_length=@(s)sqrt(1 + (3*coefficients(1)*s.^2 + 2*coefficients(2)*s + coefficients(3)).^2);
            fun_x=@(x)trapz(linspace(s_0,x,linspaceNum),curve_length(linspace(s_0,x,linspaceNum)));
            [xCoords(iterInDisplacement),~,~] = fzero(@(x)fun_x(x)-displacement(iterInDisplacement),[s_0 s_0+xend]);
        end
        r = abs((1 + (p1(1)*xCoords.^2 + p1(2)*xCoords+p1(3)).^2).^1.5) ./ (eps+abs(p2(1)*xCoords + p2(2)));
        % 计算车辆行驶过程中的横向加速度
        a_lat = v.^2 ./ (eps+r);
        max_a_lat = max(a_lat);
        if max_a_lat>alateralmax
            costList(i)=-5;
        else
            costList(i)=-CalibrationVars.wDis*minDistanceInSList/CalibrationVars.d_max+CalibrationVars.wAlat*max_a_lat/alateralmax;
        end
    end
end
%% 无碰撞换道路径集合 → 加权代价最小的最优换道路径
% 全程中与动态障碍物的最小时距
% 全程中与静态障碍物的最小距离
% 全程中的最大横向加速度
minValue = Inf;
minIndex = -1;
for i = 1:length(costList)
    if costList(i) > -1 && costList(i) < minValue
        minValue = costList(i);
        minIndex = i;
    end
end
if minIndex ~=-1
    pathPara=coefficientsList(:,minIndex);
    s_end_opt=xendList(minIndex);
    laneChangeDec=1;
else
    pathPara=zeros(4,1);
    s_end_opt=0;
    laneChangeDec=0;
end
%% 绘图
% plotFalg=1;
if plotFlag
    figure('Position', [0, 100, 1500, 100+400]) % 创建一个指定大小的图形窗口
    % xlabel('x') % 设置 x 轴标签
    % ylabel('y') % 设置 y 轴标签
    % tendList=zeros(1,round((vendmax-vendmin)/speedGap)+1);
    % xendList=tendList;
    % vendList=vendmin:interal:vendmax;
    % costList=tendList;
    % coefficientsList=zeros(4,round((vendmax-vendmin)/speedGap)+1);
    plot(sSequcence, offsetTarget2CurrentLaneSequcence, "-.");
    text(sSequcence(1)+10, offsetTarget2CurrentLaneSequcence(1)*1.05, 'centerline of target lane', 'Interpreter', 'tex')
    hold on;
    for i=1:1:length(tendList)
        xend=xendList(i);
        x = 0:0.1:xendList(i); % 生成 x 的取值范围，间隔为0.1
        y = coefficientsList(1,i)*(x.^3) + coefficientsList(2,i)*(x.^2)+ coefficientsList(3,i)*(x)+ coefficientsList(4,i); % 计算 y 的值
        if i==minIndex
            plot(x, y, 'LineWidth', 4) % 绘制fat线
        elseif costList(i) > -1 % 如果 validList 为 1，则绘制实线
            plot(x, y, 'LineWidth', 2) % 绘制实线
        else % 否则，绘制虚线
            plot(x, y, '--', 'LineWidth', 2) % 绘制虚线
        end
        % 标注 xend、yend 和 tend 的值
        %     text(xend*0.7, yend*0.8, ['xend = ', num2str(xend)])
        %     text(xend*0.7, yend*0.7, ['yend = ', num2str(yend)])
        %     text(xend*0.7, yend*0.6, ['tend = ', num2str(tend)])
        %     text(xend*0.9, yend*1.02, ['Xend = ', num2str(xend, '%.1f')])
        %     text(xend*0.7, yend*0.95, ['yend = ', num2str(yend, '%.1f')])
        %     text(xend*0.9, yend*1.05, ['Tend = ', num2str(tend, '%.1f')])
        text(xend*0.98, y(end)*1.05, ['s_{end} = ', num2str(xend, '%.1f')], 'Interpreter', 'tex')
        text(xend*0.98, y(end)*(1.05+0.05), ['t_{end} = ', num2str(tendList(i), '%.1f')], 'Interpreter', 'tex')
        text(xend*0.98, y(end)*(1.05+0.1), ['v_{end} = ', num2str(vendList(i), '%.1f')], 'Interpreter', 'tex')
        text(xend*0.98, y(end)*(1.05+0.15), ['cost = ', num2str(costList(i), '%.2f')], 'Interpreter', 'tex')

        hold on;
    end

    text(3, 0.4, ['v_{0} = ', num2str(v_0, '%.1f')], 'Interpreter', 'tex')

    xlim([0, 85]) % 设置 x 轴的范围为 0 到 100
    if offsetTarget2CurrentLane>0
        ylim([0, 4]) % 设置 x 轴的范围为 0 到 100
    else
        ylim([-4, 0]) % 设置 x 轴的范围为 0 到 100
    end
    xlabel('s(m)') % 设置 x 轴标签
    ylabel('l(m)') % 设置 y 轴标签
    for i=1:1:length(tendList)
        figure('Position', [0, 100, 1200, 100+400]) % 创建一个指定大小的图形窗口
        curve_length=@(s)sqrt(1 + (3*coefficientsList(1,i)*s.^2 + 2*coefficientsList(2,i)*s + coefficientsList(3,i)).^2);
        displacement=zeros(1,floor(tendList(i)/0.1));
        xCoords=zeros(1,floor(tendList(i)/0.1));
        for iterInDisplacement=1:1:floor(tendList(i)/0.1) % 时间数组
            displacement(iterInDisplacement) =  v_0 * (0.1*iterInDisplacement) + 0.5 * accerlMeanList(i) * (0.1*iterInDisplacement).^2; % 位移数组
            % 计算xCoords
            % fun_a = @(x)sqrt(1+((3*coefficients(1)*x.^2+4*coefficients(2)*x.^3+5*coefficients(3)*x.^4)).^2);
            % curve_length=@(s)sqrt(1 + (3*coefficients(1)*s.^2 + 2*coefficients(2)*s + coefficients(3)).^2);
            fun_x=@(x)trapz(linspace(s_0,x,linspaceNum),curve_length(linspace(s_0,x,linspaceNum)));
            %             if displacement(iterInDisplacement)>=40.0000-0.0001 && displacement(iterInDisplacement)<=40.0000+0.0001
            %                 coefficientsList(:,i)
            %             end
            %             coefficientsList(:,i)
            % displacement(iterInDisplacement)
            [xCoords(iterInDisplacement),~,~] = fzero(@(x)fun_x(x)-displacement(iterInDisplacement),[s_0 s_0+xendList(i)]);    
        end
        time2xCoords=0.1:0.1:floor(tendList(i)/0.1)*0.1;
        p0=plot(time2xCoords,xCoords,'g');
        hold on;
        for iterInRectangles = 1:length(rectangles)
            rect = rectangles{iterInRectangles};
            % 添加第一个顶点到最后一个顶点，形成一个闭合图形
            rect = [rect; rect(1,:)];
            % 绘制长方形
            p1=plot(rect(:,1), rect(:,2), '-bo');
            hold on;
        end
        for iterInRectangles = 1:length(rectanglesTargetLane)
            rect = rectanglesTargetLane{iterInRectangles};
            % 添加第一个顶点到最后一个顶点，形成一个闭合图形
            rect = [rect; rect(1,:)];
            % 绘制长方形
            p2=plot(rect(:,1), rect(:,2), '-ro');
            hold on;
        end
        % 判断departureFromCurrentLaneTimeList(i)是否大于0
        if departureFromCurrentLaneTimeList(i) > 0
            % 画垂直于横轴的直线，x=departureFromCurrentLaneTimeList(i)
            x = departureFromCurrentLaneTimeList(i);
            y = ylim; % 获取当前坐标轴的y范围
            p3=line([x x], y, 'Color', 'b'); % 画红色直线
            hold on;
        end
        % 判断instrusionIntoTargetLaneTimeList(i)是否大于0
        if instrusionIntoTargetLaneTimeList(i) > 0
            % 画垂直于横轴的直线，x=instrusionIntoTargetLaneTimeList(i)
            x = instrusionIntoTargetLaneTimeList(i);
            y = ylim; % 获取当前坐标轴的y范围
            p4=line([x x], y, 'Color', 'r'); % 画蓝色直线
        end
        if ~isempty(p1) && ~isempty(p2) && departureFromCurrentLaneTimeList(i) > 0 && instrusionIntoTargetLaneTimeList(i) > 0
            legend([p0,p1,p2,p3,p4],...,
                'trajectory of ego car', 'restricted area in current lane', 'restricted area in target lane', 'departure from the current lane', ...,
                'intrusion into the target lane', 'Location','northwest','FontSize',10);
        end
        ylabel('s(m)') % 设置 x 轴标签
        xlabel('t(s)') % 设置 y 轴标签
    end
    % costList
    %     -1：平均加速度
    %     -2：换道结束状态下的TTC
    %     -3：当前车道上的碰撞
    %     -4：检验目标车道上的碰撞
    %     -5：横向加速度
end
end



% % 已知点和斜率
% s_0 = 0; % 过点的横坐标
% gradientCurrent = 0; % 在点（s_0，0）处的斜率
% gradientTarget=1; % 在点（s_0，0）处的斜率
% xend = 1; % 过点的横坐标增量
% yend = 2; % 过点的纵坐标
% % 构建方程组矩阵
% A = [s_0^3, s_0^2, s_0, 1;
%      3*s_0^2, 2*s_0, 1, 0;
%      (s_0+xend)^3, (s_0+xend)^2, (s_0+xend), 1;
%      3*(s_0+xend)^2, 2*(s_0+xend), 1, 0];
% b = [0; gradientCurrent; yend; gradientTarget];
% % 求解方程组
% coefficients = A \ b;
%
% % 绘制曲线
% s = linspace(s_0, s_0+xend, 100); % 横坐标范围
% y = polyval(coefficients, s); % 计算纵坐标
% plot(s, y);
% xlabel('s');
% ylabel('y');
% title('三次多项式曲线');
