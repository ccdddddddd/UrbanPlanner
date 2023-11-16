function [jerk_opt,accel,velocity,length_opt] =JerkOptimalzation(S_t_lc,Speed_end,Speed_0)
A=[1 0.1 0.01/2;0 1 0.1;0 0 1];
B=[0.1^3/3 0.1^2/2 0.1]';
accel=zeros(1,21);
velocity=zeros(1,21);
velocity(1)=Speed_0;
length_opt=zeros(1,21);
A_end=0;
A_0=0;
S_0=0;
Ausloesung_1=zeros([3,23]);
Ausloesung_1(1:3,1:4)=[A B];
Gewicht=eye(23);
for i=1:1:19
    Ausloesung_1(1:3,1:(i+4))= [A*Ausloesung_1(1:3,1:(i+3)) B];
end
Ausloesung_2=[Ausloesung_1;
    eye(3) zeros(3,20);
    zeros(17,6) eye(17)];
deltaF_v=[S_t_lc;Speed_end;A_end;S_0;Speed_0;A_0];
Inverse=inv(Ausloesung_2);
G1=Inverse(:,1:6);
G2=Inverse(:,7:23);
F_xy = (G1 - (G2*(inv(G2'*Gewicht*G2))*(G2')*Gewicht*G1))*deltaF_v;
jerk_opt=F_xy(4:23);
for i=1:1:20
    accel(i+1)=accel(i)+0.1*jerk_opt(i);
    velocity(i+1)=velocity(i)+0.1*accel(i)+0.1.^2/2*jerk_opt(i);
    length_opt(i+1)=length_opt(i)+0.1*velocity(i)+0.1.^2/2*accel(i)+0.1.^3/3*jerk_opt(i);
end
end