function [R1,R2,R3,pos_start,p1,p2,pos_end]=PathPlanTurnAround(s_turnround_border,w_veh,R,D_safe,l_current,l_targetlane,l_boundry,dec2line)
l_boundry=l_boundry-dec2line;
sr1=s_turnround_border-D_safe-w_veh/2-R;
lr1=l_current+R;
if l_boundry-lr1>0
    l_p1= lr1+(l_boundry-lr1)*(R/(R+w_veh/2));
    s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2);
elseif l_boundry-lr1<0
    l_p1= lr1+(l_boundry-lr1)*((R+w_veh/2)/R);
    s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2);
else
    l_p1=lr1+R;
    s_p1=sr1;
end
% l_p1=1.5*l_targetlane-0.5*l_current;
% l_p1=l_boundry;
% s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2);
% Dpo=max(0.2,abs(w_veh/2*sin(atan2(l_p1-lr1,s_p1-sr1))));
% l_p1=l_p1-Dpo;
% s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2);
sr2=2*s_p1-sr1;
lr2=2*l_p1-lr1;
lr3=l_targetlane-R;
sr3=sr2-sqrt(4*R^2-(lr2-lr3)^2);
s_start=s_turnround_border-D_safe-w_veh/2-R;
l_start=l_current;
l_end=l_targetlane;
s_end=sr3+sqrt(R^2-(l_end-lr3)^2);
l_p2=0.5*(lr3-lr2)+lr2;
s_p2=0.5*(sr3-sr2)+sr2;
R1=[sr1,lr1];
R2=[sr2,lr2];
R3=[sr3,lr3];
theta1=atan((l_p1-lr1)/(s_p1-sr1));
% theta1=-theta1;
p1=[s_p1,l_p1,theta1 0];
theta2=(atan((l_p2-lr2)/(s_p2-sr2)));
% theta2=-theta2;
p2=[s_p2,l_p2,theta2 0];
pos_start=[s_start,l_start];
pos_end=[s_end,l_end];

end

% figure(1)
% r = R;%半径
% a1 = sr1;%圆心横坐标
% b1 = lr1;%圆心纵坐标
% theta = 0:pi/20:2*pi; %角度[0,2*pi] 
% theta1=-pi/2:pi/20:atan((l_p1-lr1)/(s_p1-sr1));
% x1 = a1+r*cos(theta);
% y1= b1+r*sin(theta);
% x11 = a1+r*cos(theta1);
% y11= b1+r*sin(theta1);
% plot(x1,y1,'g--',x11,y11,'r-')
% %plot(x1,y1,'g--',x1(s_start:s_p1),y1(l_start:l_p1),'b-')
% hold on
% a2 = sr2;%圆心横坐标
% b2 = lr2;%圆心纵坐标
% theta2=(atan((l_p1-lr2)/(s_p1-sr2))+pi):pi/20:(atan((l_p2-lr2)/(s_p2-sr2))+pi);
% x2 = a2+r*cos(theta);
% y2= b2+r*sin(theta);
% x22 = a2+r*cos(theta2);
% y22= b2+r*sin(theta2);
% plot(x2,y2,'g--',x22,y22,'r-')
% hold on 
% a3 = sr3;%圆心横坐标
% b3 = lr3;%圆心纵坐标
% theta3=atan((l_p2-lr3)/(s_p2-sr3)):pi/20:pi/2;
% x3 = a3+r*cos(theta);
% y3= b3+r*sin(theta);
% x33 = a3+r*cos(theta3);
% y33= b3+r*sin(theta3);
% plot(x3,y3,'g--',x33,y33,'r-')
% %车道线
% hold on
% %目标车道线
% plot([0,s_turnround_border],[l_targetlane,l_targetlane],'r--')
% %当前车道线
% hold on 
% plot([0,s_turnround_border],[l_current,l_current],'r--')
% %车道边界
% hold on
% plot([0,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_current-0.5*l_targetlane],'r-')
% hold on 
% plot([0,s_turnround_border],[1.5*l_targetlane-0.5*l_current,1.5*l_targetlane-0.5*l_current],'r-')
% hold on
% plot([0,s_turnround_border-6],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r-',[s_turnround_border-6,s_turnround_border],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r--')
% %掉头边界
% plot([s_turnround_border,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_targetlane-0.5*l_current],'r-')
% hold on
% %点坐标
% plot(s_start,l_start,'r*',s_end,l_end,'r*',s_p1,l_p1,'r*',s_p2,l_p2,'r*',sr1,lr1,'r.',sr2,lr2,'r.',sr3,lr3,'r.')
% axis equal
% 
