function accel=ACC(v_max,v_soll,d_ist,speed,wait,CalibrationVars)
a_max=CalibrationVars.ACC.a_max;%2.5;
a_min=CalibrationVars.ACC.a_min;%-4;
a_min_com=CalibrationVars.ACC.a_min_com;%-1.5;
tau_v_com=CalibrationVars.ACC.tau_v_com;%4;
tau_v=CalibrationVars.ACC.tau_v;%2;
tau_d=CalibrationVars.ACC.tau_d;%5;
tau_v_bre=CalibrationVars.ACC.tau_v_bre;%1;
tau_v_emg=CalibrationVars.ACC.tau_v_emg;%0.5;
tau_d_emg=CalibrationVars.ACC.tau_d_emg;%2;
t_acc=CalibrationVars.ACC.t_acc;%2;
accel=100;
accel_speedlimit=max([-2.5 (v_max-speed)/tau_v]);
if d_ist<100
    if wait==-1
        d_soll=max([speed*t_acc 17 (v_soll.^2-speed.^2)/(2*a_min) ]);
    else
        d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]);
    end
    if wait==1 
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_bre;
    elseif speed.^2/d_ist/2>-a_min_com
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_bre;
    else
        if abs(speed-v_soll)<2
            accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_com;
        else
            accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v;
        end
    end
    if d_ist<9
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d_emg)/tau_v_emg;
    elseif d_ist<12
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_emg;
    end
end
accel=min([accel accel_speedlimit]);
accel=min([accel a_max]);
accel=max([accel a_min]);
end