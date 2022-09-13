function [a_soll]=JerkLimit(a_sollpre2traj,SampleTime,a_soll)
if a_sollpre2traj~=100
    if a_soll>-2
        a_soll=median([a_sollpre2traj+2*SampleTime,a_soll,a_sollpre2traj-2*SampleTime]);
    else
        a_soll=median([a_sollpre2traj+2*SampleTime,a_soll,a_sollpre2traj-5*SampleTime]);
    end
end
end