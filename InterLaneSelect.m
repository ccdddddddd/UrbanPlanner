function TargetLane=IntersectionLaneSelection(LaneInfoSum,speed,CalibrationVars)
v_max=10;
accel=zeros(5,1);
wait=0;
TargetLane=int16(0);
for i=1:1:5
    accel(i)=ACC(v_max,LaneInfoSum(i).v_soll,LaneInfoSum(i).d_ist,speed,wait,CalibrationVars);
end
for i=1:1:5
    if abs(max(accel)-accel(i))>0.1
        LaneInfoSum(i).s=1000;
    end
end
for i=1:1:5
    if min([LaneInfoSum(1).s LaneInfoSum(2).s LaneInfoSum(3).s LaneInfoSum(4).s LaneInfoSum(5).s])==LaneInfoSum(i).s
        TargetLane=LaneInfoSum(i).LaneIndex;
    end
end
end
% 
% CalibrationVars.ACC.a_max=2.5;
% CalibrationVars.ACC.a_min=-4;
% CalibrationVars.ACC.a_min_com=-1.5;
% CalibrationVars.ACC.tau_v_com=4;
% CalibrationVars.ACC.tau_v=2;
% CalibrationVars.ACC.tau_d=5;
% CalibrationVars.ACC.tau_v_bre=1;
% CalibrationVars.ACC.tau_v_emg=0.5;
% CalibrationVars.ACC.tau_d_emg=2;
% CalibrationVars.ACC.t_acc=2;
% LaneInfo.LaneIndex=int16(0);
% LaneInfo.s=1000;
% LaneInfo.d_ist=0;
% LaneInfo.v_soll=0;
% LaneInfoSum=repmat(LaneInfo,1,5);
% LaneInfoSum(1).LaneIndex=int16(2);
% LaneInfoSum(1).s=160;
% LaneInfoSum(1).d_ist=50;
% LaneInfoSum(1).v_soll=10;
% LaneInfoSum(2).LaneIndex=int16(3);
% LaneInfoSum(2).s=150;
% LaneInfoSum(2).d_ist=50;
% LaneInfoSum(2).v_soll=9;
% LaneInfoSum(3).LaneIndex=int16(3);
% LaneInfoSum(3).s=100;
% LaneInfoSum(3).d_ist=50;
% LaneInfoSum(3).v_soll=5;
% TargetLane=IntersectionLaneSelection(LaneInfoSum,9,CalibrationVars);
