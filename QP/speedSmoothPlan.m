function [jOptSequence,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_maxVehicle,CalibrationVars)
% a_0在可以提供准确当前加速度的情况下提供，否则为默认值。MCTS决策的末状态下，有前车/需要停车的时候，提供v_end,s_end。
% x=(s-sMCTS,v,a,j) 
%% 打印及标定量赋值
% fprintf('CalibrationVars.v_min_decel = %f\n',CalibrationVars.v_min_decel);
% fprintf('CalibrationVars.accel = %f\n',CalibrationVars.accel);
% fprintf('CalibrationVars.decel = %f\n',CalibrationVars.decel);
% fprintf('CalibrationVars.numOfMaxSteps = %f\n',CalibrationVars.numOfMaxSteps);
aMax=CalibrationVars.accel;
aMin=CalibrationVars.decel;
jMax=CalibrationVars.jMax;
jMin=CalibrationVars.jMin;
offsetMax=CalibrationVars.offsetMax;
%% 计算代价函数矩阵H
% 输入参数
numOfMaxSteps=CalibrationVars.numOfMaxSteps; %50; % 最大步数
wOffset = CalibrationVars.wOffset; % 0.5; 对角线上元素的值为wOffset
wJerk = CalibrationVars.wJerk; % 0.5; % 对角线上元素的值为wJerk
% 构建矩阵
H = zeros(4*(numOfMaxSteps+1)-1);
H(1:numOfMaxSteps+1, 1:numOfMaxSteps+1) = diag(wOffset*ones(numOfMaxSteps+1, 1)/(offsetMax.^2));
H(end-numOfMaxSteps+1:end, end-numOfMaxSteps+1:end) = diag(wJerk*ones(numOfMaxSteps, 1)/(jMax.^2)); % s v a从0到50，j从0到49
sMCTS=[s_0,sMCTS];
f=zeros(4*(numOfMaxSteps+1)-1,1);
f(1:numOfMaxSteps+1)=0.5*(-2*sMCTS*wOffset)/(offsetMax.^2);
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
dt=0.1;
sDv=dt;
sDa=1/2*dt.^2;
sDj=1/6*dt.^3;
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
beq(end-1)=s_0; % -sMCTS(1);
Aeq(end,numOfMaxSteps+1+1)=1;
beq(end)=v_0;
if a_0~=-999
    Aeq=[Aeq;zeros(1,size(Aeq,2))];
    Aeq(end,(numOfMaxSteps+1)*2+1)=1;
    beq=[beq;a_0];
end
% 末状态约束
if s_end~=-999
    Aeq=[Aeq;zeros(1,size(Aeq,2))];
    Aeq(end,numOfMaxSteps+1)=1;
    beq=[beq;s_end];
end
if v_end~=-999
    Aeq=[Aeq;zeros(1,size(Aeq,2))];
    Aeq(end,(numOfMaxSteps+1)*2)=1;
    beq=[beq;v_end];
end
%% 计算上下限lb,ub   x=(s-sMCTS,v,a,j)   sMaxSequence,sMinSequence,v_maxVehicle
lb=zeros(4*(numOfMaxSteps+1)-1,1);
ub=lb;
%% vMaxSequence,vMinSequence,sMaxSequence,sMinSequence计算，借助匀加速运动（最大、最小加速度）  x=(s-sMCTS,v,a,j)
% 计算时刻
t = 0:dt:numOfMaxSteps*dt;
% 输入参数1
a = aMin; % 加速度
% 计算车速和位移
vMinSequence = a*t + v_0;
vMinSequence(vMinSequence < 0) = 0; % 如果速度小于0，则将速度设为0，即不再进行减速运动
sMinSequence = 0.5*a*t.^2 + v_0.*t + s_0;
sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
sMinSequence=max(sMinSequence,[s_0,sMinSequenceMCTS]);
% 输入参数2
a = aMax; % 加速度
% 计算车速和位移
vMaxSequence = a*t + v_0;
vMaxSequence = min(vMaxSequence, v_maxVehicle);
% vMaxSequence(vMinSequence < 0) = 0; % 如果速度小于0，则将速度设为0，即不再进行减速运动
sMaxSequence = 0.5*a*t.^2 + v_0.*t + s_0;
sMaxSequence=min(sMaxSequence,[s_0,sMaxSequenceMCTS]);
lb(1:numOfMaxSteps+1) = sMinSequence; % -sMCTS;
ub(1:numOfMaxSteps+1) = sMaxSequence; % -sMCTS)+eps;
lb(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMinSequence;
ub(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMaxSequence;
% sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
%% 计算加速度、加加速度上下限
lb(end-numOfMaxSteps+1:end) = jMin;
ub(end-numOfMaxSteps+1:end) = jMax;
lb(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMin;
ub(end-2*numOfMaxSteps:end-numOfMaxSteps) = aMax;
%% 计算起始点x0
x0=zeros(4*(numOfMaxSteps+1)-1,1);
if s_end~=-999
    vMeanExample=(s_end-s_0)/(numOfMaxSteps*dt);
    vEndExample=vMeanExample*2-v_0;
    aExample=(vEndExample-v_0)/(numOfMaxSteps*dt);
elseif a_0~=-999
    aExample=a_0;
else
    aExample=0;
end
% 初始化序列
t = 0:dt:numOfMaxSteps*dt;  % 时间序列
sExampleSequence = zeros(size(t));  % 位移序列
vExampleSequence = zeros(size(t));  % 速度序列
aExampleSequence = aExample * ones(size(t));  % 加速度序列
% 计算每个时刻下的位移和速度
for i = 1:numel(t)
    sExampleSequence(i) = s_0 + v_0 * t(i) + 0.5 * aExample * t(i)^2;
    vExampleSequence(i) = v_0 + aExample * t(i);
end
% 复制到x0
x0(1:numOfMaxSteps+1) = sExampleSequence'; % (sExampleSequence-sMCTS)';
x0(numOfMaxSteps+2:2*numOfMaxSteps+2) = vExampleSequence';
x0(end-2*numOfMaxSteps:end-numOfMaxSteps) = aExampleSequence';
%% 求解
options = optimoptions('quadprog','Display','off');
[x,~,exitflag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,x0,options);
if exitflag==1
    sOptSequence=x(1:numOfMaxSteps+1); % +sMCTS';
    vOptSequence=x(numOfMaxSteps+2:2*numOfMaxSteps+2);
    aOptSequence=x(end-2*numOfMaxSteps:end-numOfMaxSteps);
    jOptSequence=x(end-numOfMaxSteps+1:end) ;
else
    sOptSequence=zeros(1*(numOfMaxSteps+1),1);
    vOptSequence=zeros(1*(numOfMaxSteps+1),1);
    aOptSequence=zeros(1*(numOfMaxSteps+1),1);
    jOptSequence=zeros(1*(numOfMaxSteps+1)-1,1);
end
% MaxIterations
% 允许的迭代最大次数，为正整数。
% OptimalityTolerance
% 一阶最优性的终止容差：正标量。
% StepTolerance
% 关于正标量 x 的终止容差。
end

% A=[1 0.1 0.01/2;0 1 0.1;0 0 1];
% B=[0.1^3/3 0.1^2/2 0.1]';
% accel=zeros(1,21);
% velocity=zeros(1,21);
% velocity(1)=Speed_0;
% length_opt=zeros(1,21);
% A_end=0;
% A_0=0;
% S_0=0;
% Ausloesung_1=zeros([3,23]);
% Ausloesung_1(1:3,1:4)=[A B];
% Gewicht=eye(23);
% for i=1:1:19
%     Ausloesung_1(1:3,1:(i+4))= [A*Ausloesung_1(1:3,1:(i+3)) B];
% end
% Ausloesung_2=[Ausloesung_1;
%     eye(3) zeros(3,20);
%     zeros(17,6) eye(17)];
% deltaF_v=[S_t_lc;Speed_end;A_end;S_0;Speed_0;A_0];
% Inverse=inv(Ausloesung_2);
% G1=Inverse(:,1:6);
% G2=Inverse(:,7:23);
% F_xy = (G1 - (G2*(inv(G2'*Gewicht*G2))*(G2')*Gewicht*G1))*deltaF_v;
% jerk_opt=F_xy(4:23);
% for i=1:1:20
%     accel(i+1)=accel(i)+0.1*jerk_opt(i);
%     velocity(i+1)=velocity(i)+0.1*accel(i)+0.1.^2/2*jerk_opt(i);
%     length_opt(i+1)=length_opt(i)+0.1*velocity(i)+0.1.^2/2*accel(i)+0.1.^3/3*jerk_opt(i);
% end
% end

% H = [1 -1; -1 2];
% f = [-2; -6];
% A = [1 1; -1 2; 2 1];
% b = [2; 2; 3];
% options = optimoptions('quadprog','Display','iter','MaxIterations',100);
% [x,~,exitflag]  = ...
%    quadprog(H,f,A,b,[],[],[],[],[],options);
% MaxIterations
% 允许的迭代最大次数，为正整数。
% OptimalityTolerance
% 一阶最优性的终止容差：正标量。
% StepTolerance
% 关于正标量 x 的终止容差。

% % 输出结果
% disp(['时间（s）', '车速（m/s）', '位移（m）']);
% disp([num2str([t', v', s'], '%10.1f')]);

% numOfMaxSteps=2;
% s_end=-999;
% a_0=-999; % 10;
% s_0=0;
% v_0=10;
% dt=0.1;
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
% x0(1:numOfMaxSteps+1) = sExampleSequence';
% x0(numOfMaxSteps+2:2*numOfMaxSteps+2) = vExampleSequence';
% x0(end-2*numOfMaxSteps:end-numOfMaxSteps) = aExampleSequence';
% x0

% numOfMaxSteps=2;
% lb=zeros(4*(numOfMaxSteps+1)-1,1);
% ub=lb;
% dt=0.1;
% aMin=-2;
% aMax=2;
% s_0=0.1;
% v_0=0.2;
% sMinSequenceMCTS=[1.1,2.2];
% v_maxVehicle=20;
% sMaxSequenceMCTS=[11.1,12.2];
% jMin=-10;
% jMax=10;
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
% sMinSequence=min(sMinSequence,[s_0,sMinSequenceMCTS]);
% % 输入参数2
% a = aMax; % 加速度
% % 计算车速和位移
% vMaxSequence = a*t + v_0;
% vMaxSequence = min(vMaxSequence, v_maxVehicle);
% % vMaxSequence(vMinSequence < 0) = 0; % 如果速度小于0，则将速度设为0，即不再进行减速运动
% sMaxSequence = 0.5*a*t.^2 + v_0.*t + s_0;
% sMaxSequence=max(sMaxSequence,[s_0,sMaxSequenceMCTS]);
% lb(1:numOfMaxSteps+1) = sMinSequence;
% ub(1:numOfMaxSteps+1) = sMaxSequence;
% lb(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMinSequence;
% ub(numOfMaxSteps+2:2*numOfMaxSteps+2) = vMaxSequence;
% % sMinSequence(vMinSequence == 0) = s_0+(0-v_0.^2)/(2*a); % 如果速度已经降为0，则将位移设为当前位置，即不再进行减速运动
% %% 计算加速度、加加速度上下限
% lb(end-numOfMaxSteps+1:end) = jMin;
% ub(end-numOfMaxSteps+1:end) = jMax;
% lb(end-2*numOfMaxSteps+2:end-numOfMaxSteps) = aMin;
% ub(end-2*numOfMaxSteps+2:end-numOfMaxSteps) = aMax;
% lb
% ub

% numOfMaxSteps=2;
% s_0=0.1;
% v_0=0.2;
% a_0=-999; % 0.3;
% s_end=-999; % 0.4;
% v_end=-999; % 0.5;
% sMCTS=[1.1,2.2];
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
% beq(end-1)=s_0;
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
%     beq=[beq;s_end];
% end
% if v_end~=-999
%     Aeq=[Aeq;zeros(1,size(Aeq,2))];
%     Aeq(end,(numOfMaxSteps+1)*2)=1;
%     beq=[beq;v_end];
% end