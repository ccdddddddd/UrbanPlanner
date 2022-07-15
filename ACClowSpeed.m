function accel=ACClowSpeed(v_max,v_soll,d_ist,speed,CalibrationVars)
a_max=CalibrationVars.ACClowSpeed.a_max;%2.5;
a_min=CalibrationVars.ACClowSpeed.a_min;%-4;
a_min_com=CalibrationVars.ACClowSpeed.a_min_com;%-1.5;
tau_v_com=CalibrationVars.ACClowSpeed.tau_v_com;%4;
tau_v=CalibrationVars.ACClowSpeed.tau_v;%2;
tau_d=CalibrationVars.ACClowSpeed.tau_d;%5;
tau_v_bre=CalibrationVars.ACClowSpeed.tau_v_bre;%1;
tau_v_emg=CalibrationVars.ACClowSpeed.tau_v_emg;%0.5;
tau_d_emg=CalibrationVars.ACClowSpeed.tau_d_emg;%2;
tau_d_lowspeed=CalibrationVars.ACClowSpeed.tau_d_lowspeed;%5/2;
t_acc=CalibrationVars.ACClowSpeed.t_acc;%2;
d_wait=CalibrationVars.ACClowSpeed.d_wait;%4
accel=100;
accel_speedlimit=max([-2.5 (v_max-speed)/tau_v]);
if d_ist<100
%     d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]);
     d_soll=max([speed*t_acc d_wait (v_soll.^2-speed.^2)/(2*a_min) ]);
    if speed.^2/d_ist/2>-a_min_com
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_bre;
    else
        if abs(speed-v_soll)<2
            accel=(v_soll-speed+(d_ist-d_soll)/tau_d_lowspeed)/tau_v_com;
        else
            accel=(v_soll-speed+(d_ist-d_soll)/tau_d_lowspeed)/tau_v;
        end
    end
    if d_ist<d_wait
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d_emg)/tau_v_emg;
    elseif d_ist<d_wait+2
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_emg;
    end
end
accel=min([accel accel_speedlimit]);
accel=min([accel a_max]);
accel=max([accel a_min]);
end