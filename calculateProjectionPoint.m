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