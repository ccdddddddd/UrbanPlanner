function [x1,y1,theta,fval] = calculateProjectionPoint(splinedCurve,x0,y0)
    % 给定点的坐标
    point = [x0, y0];
    % 定义目标函数（垂直距离的平方）
    objectiveFunction = @(t) (point(2) - ppval(splinedCurve, t))^2+(point(1)-t)^2;
    % 最小化目标函数，得到最佳的参数值 t
    [t_optimal,fval] = fminbnd(objectiveFunction, min(splinedCurve.breaks), max(splinedCurve.breaks));
    fval=sqrt(fval);
    % 计算投影点的坐标
    x1 = t_optimal;
    y1 = ppval(splinedCurve, t_optimal);
    dy1 = ppval(fnder(splinedCurve,1), t_optimal);
    theta = atand(dy1);
end

% globalStatesBackup = frenet2global(refLine,[s_0+10,0,0,l_0+4,0,0])
% chuizuXY=refPath.closestPoint([18.1044,13.1625])
% sqrt((chuizuXY(1)-18.1044).^2+(chuizuXY(2)-13.1625).^2)
% [~,~,~,dis2lineBackup] = calculateProjectionPoint(pathLine,s_0+10,l_0+4)
% s_0+10
% l_0+4
% chuizuXY=refPath.closestPoint([26.5928,20.4552])
% sqrt((chuizuXY(1)-26.5928).^2+(chuizuXY(2)-20.4552).^2)
% 26.5928   20.4552
% [~,~,~,dis2lineBackup] = calculateProjectionPoint(pathLine,s_0,l_0)
% [~,~,~,xxx] =calculateProjectionPoint(pathLine,s_0,l_0)
