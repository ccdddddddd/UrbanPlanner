function  [aOptSequence,vOptSequence,sOptSequence,exitflag,refPath]= speedplanning(refLine,pathLine,vehicleState,v_max,ObstacleInfo,Parameter,CalibrationVars)
w_veh = Parameter.w_veh; %车宽
%CalibrationVars
pathBuffer = CalibrationVars.urbanPlanner.pathBuffer;
pathPointSpace = CalibrationVars.urbanPlanner.pathPointSpace; %2;
maxLatAcce = CalibrationVars.urbanPlanner.maxLatAcce ;% 3;
CalibMCTS = CalibrationVars.CalibMCTS;
CalibQP = CalibrationVars.CalibQP;
%速度规划参考线
fprime = fnder(pathLine,1);%一阶导
fprime2 = fnder(pathLine,2);%二阶导
s = linspace(pathLine.breaks(1),pathLine.breaks(end),round((pathLine.breaks(end)-pathLine.breaks(1))/1))';
l = ppval(pathLine,s);
dl = ppval(fprime,s);
ddl = ppval(fprime2,s); 
% pos_r = position(refLine,s);
% theta_r = tangentAngle(refLine,s);
% kappa_r = curvature(refLine,s);
% x = pos_r(:,1)-l.*sin(theta_r);
% y = pos_r(:,2)+l.*cos(theta_r);
% theta = atan2(dl,1-kappa_r.*l)+theta_r;
% refPath = referencePathFrenet([x,y,theta]);
globalStates = frenet2global(refLine,[s,zeros(length(s),1),zeros(length(s),1),l,dl,ddl]);
refPath = referencePathFrenet(globalStates(:,1:3));
frenetstate= global2frenet(refPath,[vehicleState.x,vehicleState.y,vehicleState.theta,vehicleState.k,vehicleState.speed,vehicleState.acce]);
s_0 = max(frenetstate(1),0);
v_0 = frenetstate(2);
a_0 = frenetstate(3);

%MCTS
offset = 0.5*w_veh+pathBuffer;
% obstacleMap=obstaclegraph(ObjectsInfo,pathLine,offset,s_0,s_0+150);
obstacleMap=obstacleSTgraph(ObstacleInfo,refPath,offset);

numOfPathPoints = round(refPath.PathLength/pathPointSpace);
sSequcence = linspace(0,refPath.PathLength,numOfPathPoints);
kSequcence = refPath.curvature(sSequcence')';
rSequcence = 1./kSequcence;
v_maxSequcence = max(sqrt(maxLatAcce./kSequcence),v_max);  % To be changed
[~,optimalAction]=speedCoarsePlan(obstacleMap,v_0,s_0,sSequcence,rSequcence,v_max,v_maxSequcence,CalibMCTS);
%QP
[sMCTS , speedList]= action2SMCTS(optimalAction,s_0,v_0,v_max,CalibMCTS);
sMinSequenceMCTS=zeros(1,length(sMCTS))+s_0;
sMaxSequenceMCTS=sMinSequenceMCTS+999;
v_end = speedList(end);
s_end = sMCTS(end);
v_end = -999;  % To be changed
s_end = -999;
[~,aOptSequence,vOptSequence,sOptSequence,exitflag] =speedSmoothPlan(sMCTS,v_0,s_0,a_0,v_end,s_end,sMaxSequenceMCTS,sMinSequenceMCTS,v_max,CalibQP);
if exitflag~=1
    %急减速规划
    t_list = 0:0.1:5;
    aOptSequence = zeros(1,length(t_list))-4;
    vOptSequence = v_0+aOptSequence.*t_list;
    sOptSequence = s_0 +v_0.*t_list+0.5*aOptSequence.*t_list.^2;
    %停车处理
    indexVehStop = find(vOptSequence<0,1);
    if ~isempty(indexVehStop)
        advance_s = -vOptSequence(indexVehStop-1)^2/(-4*2);
        vOptSequence(indexVehStop:end) = 0;
        sOptSequence(indexVehStop:end) = sOptSequence(indexVehStop-1)+advance_s;
    end
end
%画图
%ST
figure(3);
if isempty(gcf().Children)
    title('Planning S-T-Graph')
    xlabel('t(second)')
    ylabel('s(meter)')
else
    cla(gcf().Children)
end
t=(0:0.1:(length(sOptSequence)-1)/10)';
hold on
plot(t,sOptSequence,'*-');

xlim([0 5])
ylim([sOptSequence(1) sOptSequence(1)+90])
plot(t(2:end),sMCTS,'Color',"#EDB120",LineWidth=1);
for it = 1:length(obstacleMap)
    obstacle = obstacleMap{1,it};
    if ~isempty(obstacle)
        plot((it-1)/10,obstacle(:,1),'r.');
        plot((it-1)/10,obstacle(:,2),'r.');
    end
end

end