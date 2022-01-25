function accel=ACCcust(v_max,v_soll,d_ist,speed,a_max,a_min,t_acc)
tau_v_com=4;
tau_v=2;
tau_d=5;
if d_ist>=100
    accel=(v_max-speed)/tau_v;
else
    d_soll=max([speed*t_acc 11 (v_soll.^2-speed.^2)/(2*a_min) ]);
    if abs(speed-v_soll)<2
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_com;
    else
        accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v;
    end
end
accel=min([accel a_max]);
accel=max([accel a_min]);
if accel*0.1+speed>v_max
    accel=(v_max-speed)/0.1;
end
accel=min([accel a_max]);
accel=max([accel a_min]);
end