function pathLine = piecewisePolynomial(lineType,s0,laneChangeDirection,...
    lineleftLane_s,lineleftLane_l,lineRightLane_s,lineRightLane_l,...
    changLaneEnd_s,changLaneStart_s,changLanePara,...
    replanStart_s,replanEnd_s,replanLSequcence)
%生成路径
switch lineType
    case 'laneChange'
    if laneChangeDirection == 1
        lineTargLane_s = lineleftLane_s;
        lineTargLane_l = lineleftLane_l;
    elseif laneChangeDirection == -1
        lineTargLane_s = lineRightLane_s;
        lineTargLane_l = lineRightLane_l;
    end
    changLaneEnd_l = interp1(lineTargLane_s, lineTargLane_l, changLaneEnd_s);
    index_lineTargLane = find(lineTargLane_s>changLaneEnd_s,1);
    pathLinePart1 = spline([changLaneEnd_s,lineTargLane_s(index_lineTargLane:end)], [changLaneEnd_l,lineTargLane_l(index_lineTargLane:end)]);
    [breaks,coefs,~,~,~] = unmkpp(pathLinePart1);
    breaks=[changLaneStart_s,breaks];
    coefs = [zeros(size(coefs,1),4-size(coefs,2)),coefs];
    coefs = [changLanePara;coefs];
    pathLine = mkpp(breaks, coefs);
    case 'replanPath'
    pathLinePart1_s = linspace(replanStart_s,replanEnd_s,length(replanLSequcence));
    pathLinePart1_l = replanLSequcence;
    pathLine = spline([pathLinePart1_s,replanEnd_s+50], [pathLinePart1_l,0]);
    case 'targetLane'
    if laneChangeDirection == 1
        lineTargLane_s = lineleftLane_s;
        lineTargLane_l = lineleftLane_l;
    elseif laneChangeDirection == -1
        lineTargLane_s = lineRightLane_s;
        lineTargLane_l = lineRightLane_l;
    end
    l_0 = interp1(lineTargLane_s, lineTargLane_l, s_0);
    index_targLane = find(lineTargLane_s >= s_0,1);
    pathLine = spline([s0,lineTargLane_s(index_targLane:end)], [l_0,lineTargLane_l(index_targLane:end)]);
    case 'currentLane'
    pathLine = spline([s0,s0+150], [0,0]);
end
end