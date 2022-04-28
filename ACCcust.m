function accel=ACCcust(v_max,v_soll,d_ist,speed,a_max,a_min,t_acc,CalibrationVars)
tau_v_com=CalibrationVars.ACCcust.tau_v_com;%4;
tau_v=CalibrationVars.ACCcust.tau_v;%2;
tau_d=CalibrationVars.ACCcust.tau_d;%5;
accel=100;
accel_speedlimit=max([-2.5 (v_max-speed)/tau_v]);

if d_ist<100
    d_soll=max([speed*t_acc 11 (v_soll.^2-speed.^2)/(2*a_min) ]);
    if abs(speed-v_soll)<2
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_com;
    else
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v;
    end
end
accel=min([accel accel_speedlimit]);
accel=min([accel a_max]);
accel=max([accel a_min]);
end