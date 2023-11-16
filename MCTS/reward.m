function r=reward(stateFather,stateSon,CalibrationVars,v_maxVehicle)
potentialOfSon=CalibrationVars.w1*min(1,stateSon.minDistance/CalibrationVars.d_max)+...,
    CalibrationVars.w2*min(1,stateSon.speed/v_maxVehicle);% w1=0.5;w2=0.5;v_maxVehicle=20;d_max=60=v_maxVehicle*(t_re+t_follow)
potentialOfFather=CalibrationVars.w1*min(1,stateFather.minDistance/CalibrationVars.d_max)+...,
    CalibrationVars.w2*min(1,stateFather.speed/v_maxVehicle);
r=potentialOfSon-potentialOfFather;
end
