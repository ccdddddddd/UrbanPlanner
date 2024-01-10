function pathLine = piecewisePolynomial(lineType,refLine,s0,laneChangeDirection,...
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
        %曲线平移
        pathPara = changLanePara;
        delta = -changLaneStart_s;
        pathPara_shifted = pathPara;
        pathPara_shifted(1) =  pathPara(1);
        pathPara_shifted(2) = pathPara(1)*(-3*delta) + pathPara(2);
        pathPara_shifted(3) =  pathPara(1)*3*delta^2  + pathPara(2)*(-2*delta) + pathPara(3);
        pathPara_shifted(4) = pathPara(1)*(-delta^3) + pathPara(2)*delta^2 - pathPara(3)*delta + pathPara(4);
        %新曲线系数pathPara_shifted
        coefs = [pathPara_shifted';coefs];
        pathLine = mkpp(breaks, coefs);
        %画图
        %     x1 = 0:0.1:100; % 生成 x 的取值范围，间隔为0.1
        %     y1 = pathPara(1)*(x1.^3) + pathPara(2)*(x1.^2)+ pathPara(3)*(x1)+ pathPara(4); % 计算 y 的值
        %     plot(x1,y1,'y')
        %     hold on
        %     y2 = pathPara_shifted(1)*(x1.^3) + pathPara_shifted(2)*(x1.^2)+ pathPara_shifted(3)*(x1)+ pathPara_shifted(4); % 计算 y 的值
        %     plot(x1,y2,'r')
        %     hold on

    case 'replanPath'
        pathLinePart1_s = linspace(replanStart_s,replanEnd_s,length(replanLSequcence));
        pathLinePart1_l = replanLSequcence';
        if replanEnd_s + 1< 150
            %直线段
            x = linspace(replanEnd_s + 1,150,round((150-(replanEnd_s + 1))/2));
            y = zeros(1,round((150-(replanEnd_s + 1))/2));
            pathLine = spline([pathLinePart1_s,x], [pathLinePart1_l,y]);
        else
            pathLine = spline(pathLinePart1_s, pathLinePart1_l);
        end
    case 'targetLane'
        if laneChangeDirection == 1
            lineTargLane_s = lineleftLane_s;
            lineTargLane_l = lineleftLane_l;
        elseif laneChangeDirection == -1
            lineTargLane_s = lineRightLane_s;
            lineTargLane_l = lineRightLane_l;
        end
        l0 = interp1(lineTargLane_s, lineTargLane_l, s0);
        index_targLane = find(lineTargLane_s > s0,1);
        pathLine = spline([s0,lineTargLane_s(index_targLane:end)], [l0,lineTargLane_l(index_targLane:end)]);
    case 'currentLane'
        remainLength = refLine.PathLength - s0;
        pathLine = spline([s0,s0+min(remainLength,150)], [0,0]);
end
end