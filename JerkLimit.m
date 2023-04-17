function [a_soll]=JerkLimit(a_sollpre2traj,SampleTime,a_soll,CalibrationVars)
jerkLimit = CalibrationVars.UrbanPlanner.jerkLimit;
if a_sollpre2traj~=100
    if a_soll>-2
        a_soll=median([a_sollpre2traj+jerkLimit*SampleTime,a_soll,a_sollpre2traj-jerkLimit*SampleTime]);
    else
        a_soll=median([a_sollpre2traj+jerkLimit*SampleTime,a_soll,a_sollpre2traj-2.5*jerkLimit*SampleTime]);
    end
end
end