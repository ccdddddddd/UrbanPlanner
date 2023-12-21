function [sList , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_maxVehicle)
speed = v_0; % 当前车速
s = s_0; % 当前位置
% v_max = v_maxVehicle; % 最高车速
% 计算每一帧的车速和位置
dt = 0.1; % 每一帧的时间间隔
frames = 50; % 总共计算的帧数
speedList=zeros(frames,1);
sList=zeros(1,frames);
timeList=0.1:0.1:5;
for i = 1:1:frames
    % 判断当前状态（加速、匀速、减速、静止）
    actionNum=ceil(i/5);
    action=optimalAction(actionNum);
    a=0;
    switch action
        case 1
            a=decel;
        case 2
            a=decelComfort;
        case 3
            a=0;
        case 4
            a=accelComfort;
        case 5
            a=accel;
    end
    speedNew = median([speed + a * dt,0,v_maxVehicle]);
    s = s + (speed+speedNew)/2 * dt;
    % 输出当前帧的车速和位置
    speedList(i)=speedNew;
    sList(i)=s;
    speed=speedNew;
end

end