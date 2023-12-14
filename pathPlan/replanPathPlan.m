function [lSequcence,exitflag] = replanPathPlan(s_0,s_end,l_0,l_end,headingEnd,speed,sSequcence,lRefSequcence,rSequcence,headingCurrent,offsetRight2CurrentLane,offsetLeft2CurrentLane,...,
    CalibrationVars,BasicInfo) 
% lSequcence为l坐标序列
%% 标定量和初始化赋值
ds=CalibrationVars.ds; %2; % 最大步数
if s_end~=-999
    numOfMaxSteps=min(round((s_end-s_0)/ds),CalibrationVars.numOfMaxSteps); %50; % 最大步数
else
    numOfMaxSteps=CalibrationVars.numOfMaxSteps; %50; % 最大步数
end
lDer_0=tand(headingCurrent);
%% 计算代价函数矩阵H
% 输入参数
wOffset = CalibrationVars.wOffset; % 0.4;
wOffsetDer = CalibrationVars.wOffsetDer; % 0.2; 
wOffsetSecDer = CalibrationVars.wOffsetSecDer; % 0.2; 
wOffsetThrDer = CalibrationVars.wOffsetThrDer; % 0.2; 
% offsetMax=CalibrationVars.offsetMax; % ;
% offsetDerMax=CalibrationVars.offsetDerMax; % ; 
% offsetSecDerMax=CalibrationVars.offsetSecDerMax; % ; 
% offsetThrDerMax=CalibrationVars.offsetThrDerMax; % ; 
offsetMax=1; % ;
offsetDerMax=1; % ; 
offsetSecDerMax=1; % ; 
offsetThrDerMax=1; % ; 

offSetSequence=zeros(numOfMaxSteps+1,1);
s = 0:ds:numOfMaxSteps*ds;
aMin=zeros(numOfMaxSteps+1,1);
aMax=zeros(numOfMaxSteps+1,1);
for i=1:1:numOfMaxSteps+1
    % 输入要插值的s坐标
    % s = 2.5;
    % 找到stateNew.s所在的区间
    index = find(sSequcence <= s_0+s(i), 1, 'last');
    % 线性插值
    if ~isempty(index) && index < numel(sSequcence)
        s1 = sSequcence(index);
        s2 = sSequcence(index + 1);
        r1 = rSequcence(index);
        r2 = rSequcence(index + 1);
        l1 = lRefSequcence(index);
        l2 = lRefSequcence(index + 1);
        interpolated_r = r1 + (r2 - r1) * (s_0+s(i) - s1) / (s2 - s1);
        interpolated_l = l1 + (l2 - l1) * (s_0+s(i) - s1) / (s2 - s1);
    else
        fprintf('error:s坐标超出范围\n');
        interpolated_r=100000;
        interpolated_l = 0;
    end
    %         % 输出插值结果
    %         interpolated_r
    aMin(i)=-tan(BasicInfo.maxSteerAngle / BasicInfo.steerRatio) / BasicInfo.wheelBase-1/interpolated_r;
    aMax(i)=tan(BasicInfo.maxSteerAngle / BasicInfo.steerRatio) / BasicInfo.wheelBase-1/interpolated_r;
    offSetSequence(i)=interpolated_l;
end
% 构建矩阵
H = zeros(4*(numOfMaxSteps+1)-1);
H(1:numOfMaxSteps+1, 1:numOfMaxSteps+1) = diag(wOffset*ones(numOfMaxSteps+1, 1)/(offsetMax.^2));
H(numOfMaxSteps+2:2*numOfMaxSteps+2, numOfMaxSteps+2:2*numOfMaxSteps+2) = diag(wOffsetDer*ones(numOfMaxSteps+1, 1)/(offsetDerMax.^2));
H(2*numOfMaxSteps+3:3*numOfMaxSteps+3, 2*numOfMaxSteps+3:3*numOfMaxSteps+3) = diag(wOffsetSecDer*ones(numOfMaxSteps+1, 1)/(offsetSecDerMax.^2));
H(end-numOfMaxSteps+1:end, end-numOfMaxSteps+1:end) = diag(wOffsetThrDer*ones(numOfMaxSteps, 1)/(offsetThrDerMax.^2)); % s v a从0到50，j从0到49
f=zeros(4*(numOfMaxSteps+1)-1,1);
f(1:numOfMaxSteps+1)=2*(-2*offSetSequence*wOffset);
% 输出结果
% disp(H);
%% 计算等式约束Aeq,beq    x=(s-sMCTS,v,a,j)
% s(k+1)=s(k)+dt*v(k)+1/2*dt.^2*a(k)+1/3*dt.^3*j(k)
% x(k+1+1)+sMCTS(k+1)=x(k+1)+sMCTS(k)+sDv*x(numOfMaxSteps+1+k+1)+sDa*x[(numOfMaxSteps+1)*2+k+1]+sDj*x[(numOfMaxSteps+1)*3+k+1]
% sMCTS(k+1)-sMCTS(k)=x(k+1)-x(k+1+1)+sDv*x(numOfMaxSteps+1+k+1)+sDa*x[(numOfMaxSteps+1)*2+k+1]+sDj*x[(numOfMaxSteps+1)*3+k+1]

% v(k+1)=v(k)+dt*a(k)+1/2*dt.^2*j(k)
% x(numOfMaxSteps+1+k+1+1)=x(numOfMaxSteps+1+k+1)+sDv*x[(numOfMaxSteps+1)*2+k+1]+sDa*x[(numOfMaxSteps+1)*3+k+1]
% 0=x(numOfMaxSteps+1+k+1)-x(numOfMaxSteps+1+k+1+1)+sDv*x[(numOfMaxSteps+1)*2+k+1]+sDa*x[(numOfMaxSteps+1)*3+k+1]

% a(k+1)=a(k)+dt*j(k)
% x[(numOfMaxSteps+1)*2+k+2]=x[(numOfMaxSteps+1)*2+k+1]+sDv*x[(numOfMaxSteps+1)*3+k+1]
% 0=x[(numOfMaxSteps+1)*2+k+1]-x[(numOfMaxSteps+1)*2+k+2]+sDv*x[(numOfMaxSteps+1)*3+k+1]

% s(k+1)=x(k+1+1)+sMCTS(k+1) s(k)=x(k+1)+sMCTS(k)
% v(k)=x(numOfMaxSteps+1+k+1)
% a(k)=x[(numOfMaxSteps+1)*2+k+1]
% j(k)=x[(numOfMaxSteps+1)*3+k+1]
% sMCTS=[s_0,sMCTS];
if headingEnd~=-999
    lDer_end=tand(headingEnd);
else
    lDer_end=-999;
end
sDv=ds;
sDa=1/2*ds.^2;
sDj=1/6*ds.^3;
Aeq=zeros(3*numOfMaxSteps+2,4*(numOfMaxSteps+1)-1);
% a_0,v_end,s_end不一定存在，默认值为-999，存在就不是-999 % (a_0~=-999)+(v_end~=-999)+(s_end~=-999)
beq=zeros(size(Aeq,1),1);
% 连续性约束
for k=0:1:numOfMaxSteps-1
    % 位移连续
    beq(k*3+1)=0; % sMCTS(k+2)-sMCTS(k+1);
    Aeq(k*3+1,k+1)=1;
    Aeq(k*3+1,k+2)=-1;
    Aeq(k*3+1,numOfMaxSteps+1+k+1)=sDv;
    Aeq(k*3+1,(numOfMaxSteps+1)*2+k+1)=sDa;
    Aeq(k*3+1,(numOfMaxSteps+1)*3+k+1)=sDj;
    % 速度连续
    beq(k*3+2)=0;
    Aeq(k*3+2,numOfMaxSteps+1+k+1)=1;
    Aeq(k*3+2,numOfMaxSteps+1+k+1+1)=-1;
    Aeq(k*3+2,(numOfMaxSteps+1)*2+k+1)=sDv;
    Aeq(k*3+2,(numOfMaxSteps+1)*3+k+1)=sDa;
    % 加速度连续
    beq(k*3+3)=0;
    Aeq(k*3+3,(numOfMaxSteps+1)*2+k+1)=1;
    Aeq(k*3+3,(numOfMaxSteps+1)*2+k+2)=-1;
    Aeq(k*3+3,(numOfMaxSteps+1)*3+k+1)=sDv;
end
% 初始状态约束
Aeq(end-1,1)=1;
beq(end-1)=l_0; % -sMCTS(1);
Aeq(end,numOfMaxSteps+1+1)=1;
beq(end)=lDer_0;
% 末状态约束
if l_end~=-999
    Aeq=[Aeq;zeros(1,size(Aeq,2))];
    Aeq(end,numOfMaxSteps+1)=1;
    beq=[beq;l_end];
end
if lDer_end~=-999
    Aeq=[Aeq;zeros(1,size(Aeq,2))];
    Aeq(end,(numOfMaxSteps+1)*2)=1;
    beq=[beq;lDer_end];
end
%% 计算上下限lb,ub   x=(s-sMCTS,v,a,j)   sMaxSequence,sMinSequence,v_maxVehicle
FLAGS_lateral_derivative_bound_default=CalibrationVars.FLAGS_lateral_derivative_bound_default;
MovingRoomFromCenterLane=CalibrationVars.MovingRoomFromCenterLane;
widthOfVehicle=BasicInfo.widthOfVehicle;
lb=zeros(4*(numOfMaxSteps+1)-1,1);
ub=lb;
%% 零阶导数约束
lb(1:numOfMaxSteps+1) = -offsetRight2CurrentLane+MovingRoomFromCenterLane+0.5*widthOfVehicle;
ub(1:numOfMaxSteps+1) = offsetLeft2CurrentLane-MovingRoomFromCenterLane-0.5*widthOfVehicle;
lb(numOfMaxSteps+2:2*numOfMaxSteps+2) = -FLAGS_lateral_derivative_bound_default;
ub(numOfMaxSteps+2:2*numOfMaxSteps+2) = FLAGS_lateral_derivative_bound_default;
% sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
%% 计算加速度、加加速度上下限
lb(end-numOfMaxSteps+1:end) = -BasicInfo.maxSteerAngleRate / BasicInfo.steerRatio / 2 / BasicInfo.wheelBase / speed;
ub(end-numOfMaxSteps+1:end) = BasicInfo.maxSteerAngleRate / BasicInfo.steerRatio / 2 / BasicInfo.wheelBase / speed;
lb(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMin;
ub(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMax;
%% 计算起始点x0
x0=zeros(4*(numOfMaxSteps+1)-1,1);
if l_end~=-999
    l_end4x0=l_end;
else
    l_end4x0=0;
end
if lDer_end~=-999
    lDer_end4x0=lDer_end;
else
    lDer_end4x0=0;
end
xend=(numOfMaxSteps)*ds;
A = [0, 0, 0, 1;
    0, 0, 1, 0;
    (xend)^3, (xend)^2, (xend), 1;
    3*(xend)^2, 2*(xend), 1, 0];
b = [l_0; lDer_0; l_end4x0; lDer_end4x0];
% 求解方程组
coefficients = A \ b;
% 初始化序列
% s = 0:ds:numOfMaxSteps*ds;  % 时间序列
sExampleSequence = zeros(size(s));  % 位移序列
vExampleSequence = zeros(size(s));  % 速度序列
aExampleSequence = zeros(size(s));  % 加速度序列
% 计算每个时刻下的位移和速度
for i = 1:numel(s)
    x=s(i);
    sExampleSequence(i) = coefficients(1)*(x.^3) + coefficients(2)*(x.^2)+ coefficients(3)*(x)+ coefficients(4);
    vExampleSequence(i) = coefficients(1)*(3*x.^2) + coefficients(2)*(2*x)+ coefficients(3);
    aExampleSequence(i) = coefficients(1)*(6*x) + coefficients(2)*(2);  % 加速度序列
end
% 复制到x0
x0(1:numOfMaxSteps+1) = sExampleSequence';
x0(numOfMaxSteps+2:2*numOfMaxSteps+2) = vExampleSequence';
x0(end-2*numOfMaxSteps:end-numOfMaxSteps) = aExampleSequence';
x0(end-numOfMaxSteps+1:end) = coefficients(1)*6;

% 绘制曲线
% syms x
% y = coefficients(4) + coefficients(3)*x + coefficients(2)*x^2 + coefficients(1)*x^3;
% dy = diff(y, x);
% dyy = diff(dy, x);
% dyyy = diff(dyy, x);
% fplot(y, [0, xend], 'DisplayName', '三次多项式曲线');
% hold on
% fplot(dy, [0, xend], 'DisplayName', '一阶导数曲线');
% fplot(dyy, [0, xend], 'DisplayName', '二阶导数曲线');
% fplot(dyyy, [0, xend], 'DisplayName', '三阶导数曲线');
% legend

%% 求解
options = optimoptions('quadprog','Display','off');
[x,~,exitflag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,x0,options);
if exitflag==1
    lSequcence=x(1:numOfMaxSteps+1);
else
    lSequcence=zeros(1*(numOfMaxSteps+1),1);
end
end



% %% 打印及标定量赋值
% % fprintf('CalibrationVars.v_min_decel = %f\n',CalibrationVars.v_min_decel);
% % fprintf('CalibrationVars.accel = %f\n',CalibrationVars.accel);
% % fprintf('CalibrationVars.decel = %f\n',CalibrationVars.decel);
% % fprintf('CalibrationVars.numOfMaxSteps = %f\n',CalibrationVars.numOfMaxSteps);
% aMax=CalibrationVars.accel;
% aMin=CalibrationVars.decel;
% jMax=CalibrationVars.jMax;
% jMin=CalibrationVars.jMin;
% offsetMax=CalibrationVars.offsetMax;
% %% 计算代价函数矩阵H
% % 输入参数
% numOfMaxSteps=CalibrationVars.numOfMaxSteps; %50; % 最大步数
% wOffset = CalibrationVars.wOffset; % 0.5; 对角线上元素的值为wOffset
% wJerk = CalibrationVars.wJerk; % 0.5; % 对角线上元素的值为wJerk
% % 构建矩阵
% H = zeros(4*(numOfMaxSteps+1)-1);
% H(1:numOfMaxSteps+1, 1:numOfMaxSteps+1) = diag(wOffset*ones(numOfMaxSteps+1, 1)/(offsetMax.^2));
% H(end-numOfMaxSteps+1:end, end-numOfMaxSteps+1:end) = diag(wJerk*ones(numOfMaxSteps, 1)/(jMax.^2)); % s v a从0到50，j从0到49
% % 输出结果
% % disp(H);
% %% 计算等式约束Aeq,beq    x=(s-sMCTS,v,a,j)
% % s(k+1)=s(k)+dt*v(k)+1/2*dt.^2*a(k)+1/3*dt.^3*j(k)
% % x(k+1+1)+sMCTS(k+1)=x(k+1)+sMCTS(k)+sDv*x(numOfMaxSteps+1+k+1)+sDa*x[(numOfMaxSteps+1)*2+k+1]+sDj*x[(numOfMaxSteps+1)*3+k+1]
% % sMCTS(k+1)-sMCTS(k)=x(k+1)-x(k+1+1)+sDv*x(numOfMaxSteps+1+k+1)+sDa*x[(numOfMaxSteps+1)*2+k+1]+sDj*x[(numOfMaxSteps+1)*3+k+1]
% 
% % v(k+1)=v(k)+dt*a(k)+1/2*dt.^2*j(k)
% % x(numOfMaxSteps+1+k+1+1)=x(numOfMaxSteps+1+k+1)+sDv*x[(numOfMaxSteps+1)*2+k+1]+sDa*x[(numOfMaxSteps+1)*3+k+1]
% % 0=x(numOfMaxSteps+1+k+1)-x(numOfMaxSteps+1+k+1+1)+sDv*x[(numOfMaxSteps+1)*2+k+1]+sDa*x[(numOfMaxSteps+1)*3+k+1]
% 
% % a(k+1)=a(k)+dt*j(k)
% % x[(numOfMaxSteps+1)*2+k+2]=x[(numOfMaxSteps+1)*2+k+1]+sDv*x[(numOfMaxSteps+1)*3+k+1]
% % 0=x[(numOfMaxSteps+1)*2+k+1]-x[(numOfMaxSteps+1)*2+k+2]+sDv*x[(numOfMaxSteps+1)*3+k+1]
% 
% % s(k+1)=x(k+1+1)+sMCTS(k+1) s(k)=x(k+1)+sMCTS(k)
% % v(k)=x(numOfMaxSteps+1+k+1)
% % a(k)=x[(numOfMaxSteps+1)*2+k+1]
% % j(k)=x[(numOfMaxSteps+1)*3+k+1]
% sMCTS=[s_0,sMCTS];
% dt=0.1;
% sDv=dt;
% sDa=1/2*dt.^2;
% sDj=1/6*dt.^3;
% Aeq=zeros(3*numOfMaxSteps+2,4*(numOfMaxSteps+1)-1);
% % a_0,v_end,s_end不一定存在，默认值为-999，存在就不是-999 % (a_0~=-999)+(v_end~=-999)+(s_end~=-999)
% beq=zeros(size(Aeq,1),1);
% % 连续性约束
% for k=0:1:numOfMaxSteps-1
%     % 位移连续
%     beq(k*3+1)=sMCTS(k+2)-sMCTS(k+1);
%     Aeq(k*3+1,k+1)=1;
%     Aeq(k*3+1,k+2)=-1;
%     Aeq(k*3+1,numOfMaxSteps+1+k+1)=sDv;
%     Aeq(k*3+1,(numOfMaxSteps+1)*2+k+1)=sDa;
%     Aeq(k*3+1,(numOfMaxSteps+1)*3+k+1)=sDj;
%     % 速度连续
%     beq(k*3+2)=0;
%     Aeq(k*3+2,numOfMaxSteps+1+k+1)=1;
%     Aeq(k*3+2,numOfMaxSteps+1+k+1+1)=-1;
%     Aeq(k*3+2,(numOfMaxSteps+1)*2+k+1)=sDv;
%     Aeq(k*3+2,(numOfMaxSteps+1)*3+k+1)=sDa;
%     % 加速度连续
%     beq(k*3+3)=0;
%     Aeq(k*3+3,(numOfMaxSteps+1)*2+k+1)=1;
%     Aeq(k*3+3,(numOfMaxSteps+1)*2+k+2)=-1;
%     Aeq(k*3+3,(numOfMaxSteps+1)*3+k+1)=sDv;
% end
% % 初始状态约束
% Aeq(end-1,1)=1;
% beq(end-1)=s_0-sMCTS(1);
% Aeq(end,numOfMaxSteps+1+1)=1;
% beq(end)=v_0;
% if a_0~=-999
%     Aeq=[Aeq;zeros(1,size(Aeq,2))];
%     Aeq(end,(numOfMaxSteps+1)*2+1)=1;
%     beq=[beq;a_0];
% end
% % 末状态约束
% if s_end~=-999
%     Aeq=[Aeq;zeros(1,size(Aeq,2))];
%     Aeq(end,numOfMaxSteps+1)=1;
%     beq=[beq;s_end-sMCTS(end)];
% end
% if v_end~=-999
%     Aeq=[Aeq;zeros(1,size(Aeq,2))];
%     Aeq(end,(numOfMaxSteps+1)*2)=1;
%     beq=[beq;v_end];
% end
% %% 计算上下限lb,ub   x=(s-sMCTS,v,a,j)   sMaxSequence,sMinSequence,v_maxVehicle
% lb=zeros(4*(numOfMaxSteps+1)-1,1);
% ub=lb;
% %% vMaxSequence,vMinSequence,sMaxSequence,sMinSequence计算，借助匀加速运动（最大、最小加速度）  x=(s-sMCTS,v,a,j)
% % 计算时刻
% t = 0:dt:numOfMaxSteps*dt;
% % 输入参数1
% a = aMin; % 加速度
% % 计算车速和位移
% vMinSequence = a*t + v_0;
% vMinSequence(vMinSequence < 0) = 0; % 如果速度小于0，则将速度设为0，即不再进行减速运动
% sMinSequence = 0.5*a*t.^2 + v_0.*t + s_0;
% sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
% sMinSequence=max(sMinSequence,[s_0,sMinSequenceMCTS]);
% % 输入参数2
% a = aMax; % 加速度
% % 计算车速和位移
% vMaxSequence = a*t + v_0;
% vMaxSequence = min(vMaxSequence, v_maxVehicle);
% % vMaxSequence(vMinSequence < 0) = 0; % 如果速度小于0，则将速度设为0，即不再进行减速运动
% sMaxSequence = 0.5*a*t.^2 + v_0.*t + s_0;
% sMaxSequence=min(sMaxSequence,[s_0,sMaxSequenceMCTS]);
% lb(1:numOfMaxSteps+1) = sMinSequence-sMCTS;
% ub(1:numOfMaxSteps+1) = sMaxSequence-sMCTS;
% lb(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMinSequence;
% ub(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMaxSequence;
% % sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
% %% 计算加速度、加加速度上下限
% lb(end-numOfMaxSteps+1:end) = jMin;
% ub(end-numOfMaxSteps+1:end) = jMax;
% lb(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMin;
% ub(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMax;
% %% 计算起始点x0
% x0=zeros(4*(numOfMaxSteps+1)-1,1);
% if s_end~=-999
%     vMeanExample=(s_end-s_0)/(numOfMaxSteps*dt);
%     vEndExample=vMeanExample*2-v_0;
%     aExample=(vEndExample-v_0)/(numOfMaxSteps*dt);
% elseif a_0~=-999
%     aExample=a_0;
% else
%     aExample=0;
% end
% % 初始化序列
% t = 0:dt:numOfMaxSteps*dt;  % 时间序列
% sExampleSequence = zeros(size(t));  % 位移序列
% vExampleSequence = zeros(size(t));  % 速度序列
% aExampleSequence = aExample * ones(size(t));  % 加速度序列
% % 计算每个时刻下的位移和速度
% for i = 1:numel(t)
%     sExampleSequence(i) = s_0 + v_0 * t(i) + 0.5 * aExample * t(i)^2;
%     vExampleSequence(i) = v_0 + aExample * t(i);
% end
% % 复制到x0
% x0(1:numOfMaxSteps+1) = (sExampleSequence-sMCTS)';
% x0(numOfMaxSteps+2:2*numOfMaxSteps+2) = vExampleSequence';
% x0(end-2*numOfMaxSteps:end-numOfMaxSteps) = aExampleSequence';
% %% 求解
% options = optimoptions('quadprog','Display','off');
% [x,~,exitflag] = quadprog(H,[],[],[],Aeq,beq,lb,ub,x0,options);
% if exitflag==1
%     sOptSequence=x(1:numOfMaxSteps+1)+sMCTS';
%     vOptSequence=x(numOfMaxSteps+2:2*numOfMaxSteps+2);
%     aOptSequence=x(end-2*numOfMaxSteps:end-numOfMaxSteps);
%     jOptSequence=x(end-numOfMaxSteps+1:end) ;
% else
%     sOptSequence=zeros(1*(numOfMaxSteps+1),1);
%     vOptSequence=zeros(1*(numOfMaxSteps+1),1);
%     aOptSequence=zeros(1*(numOfMaxSteps+1),1);
%     jOptSequence=zeros(1*(numOfMaxSteps+1)-1,1);
% end