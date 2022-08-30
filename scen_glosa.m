function [vOpt, vgMin, vgMax] = scen_glosa(d, v0, tl, vMax, vMin, a0, dec, mrg, desRate, dIntxn, dMin)
    vOpt = -1;
    vgMin = -1;
    vgMax = -1;
    if -dIntxn <= d && d < 0
        if tl(1) > 0
            vOpt = min(vMin + 0.7*(vMax-vMin), vMax);
            vgMax = vMax;
            vgMin = vMin;
        else
            vOpt = 0;
        end
        return
    end
    dv = 0.25;
    vn = floor(33/dv); % 限速33m/s, 离散间隔0.25m/s
    vList = zeros(vn, 1);
    tp = zeros(vn, 1);
    for i = 1 : vn
        v1 = i*dv;
        a = a0;
        if v1 < v0
            a = -a;
        end
        s = (v1^2 - v0^2) /a /2;
        if s <= d
            tp(i) = (v1 - v0)/a + (d - s)/v1;
        else
            v2 = sqrt(v0^2 + 2*a*d);
            tp(i) = (v2 - v0)/a;
        end
    end
    [tp, idx] = sort(tp);
    ts = 0;
    k = 1;
    for i = 1 : vn
        while(tl(k) ~= 0 && ts + abs(tl(k)) <= tp(i))
            ts = ts + abs(tl(k));
            k = k + 1;
        end
        if tl(k) == 0
            break
        end
        if tl(k) > 0
            vList(idx(i)) = 1;
        end
    end
    % 去掉限速外车速
    for i = 1 : floor(vMin/dv)
        vList(i) = 0;
    end
    for i = floor(vMax/dv)+1 : vn
        vList(i) = 0;
    end   
%     close;plot([1:vn]*dv, vList, 'linewidth', 2);
    
    % ----------------------- 找最佳车速与区间 ----------------------------
    % 可行车速需要可行车速区间大小超阈值，设0为不启用
    % 情况1，如果当前车速在可行区间
    k = floor(v0/dv);
    if k > 0 && vList(k) == 1
        i = k;
        while i > 1 && vList(i-1) == 1
            i = i - 1;
        end
        j = k;
        while j < vn && vList(j+1) == 1
            j = j + 1;
        end
%         if k - i >= mrg && j - k >= mrg
%             vOpt = v0;
%             vgMin = i*dv;
%             vgMax = j*dv;
%             return
%         else
        if j - i >= mrg
            vOpt = i*dv + (j-i)*desRate*dv;
            vgMin = i*dv;
            vgMax = j*dv;
            return
        end
    end
    % 情况2，另找可行区间
    k = min(vn, floor(vMax/dv));
    while k >= 1
        if vList(k) == 0
            k = k - 1;
            continue
        end
        i = k;
        while i > 1 && vList(i-1) == 1
            i = i - 1;
        end
        if k - i >= mrg
            vOpt = i*dv + (k-i)*desRate*dv;
            vgMin = i*dv;
            vgMax = k*dv;
            return
        else
            k = k - 1;
        end
    end
    
    % 不能通行的速度规划
    if d > dMin
        vstop = min(sqrt(2*dec*(d-dMin)), vMax);
        if v0 >= vstop
            vOpt = vstop;
        else
            vOpt = -1;
        end
    else
        vOpt = 0;
    end
end
