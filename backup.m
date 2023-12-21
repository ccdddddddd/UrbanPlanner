%-------------------------------------------------------------------
if %换道中
    %计算与换道路径偏差
    if %与现有换道路径偏差过大
        if %位于目标车道
            %重规划2目标车道
            %速度规划
            if  %速度ok  
                %生成轨迹
            else
                %生成紧急制动轨迹
            end
        else
            %重换道+速度规划
            if %换道ok
                %速度规划
                if %速度规划ok
                    %生成轨迹
                end
                if %换道不ok || %速度规划不ok
                    %重规划原车道+ 速度规划
                    %速度规划
                    if %速度ok
                        % 生成轨迹
                    else
                        %紧急制动轨迹
                    end
                end
            end
        end
    else
        %旧换道路径速度规划
        if exitflag == 1%速度规划成功
            %生成轨迹
        elseif %位于目标车道
            %生成紧急制动轨迹
        else
            %重换道+速度规划
            if %换道ok
                %速度规划
                if %速度规划ok
                    %生成轨迹
                end
                if %换道不ok || %速度规划不ok
                    %重规划原车道+ 速度规划
                    %速度规划
                    if exitflag ==1 %速度ok
                        % 生成轨迹
                    else
                        %紧急制动轨迹
                    end
                end
            end
        end
    end
elseif isreplanPath == 2 %重规划目标车道中
    %速度规划
    if %速度规划ok
        %生成轨迹
    else
        %紧急制动轨迹
    end 
else
    if %需换道
        %换道路径
        if %换道路径ok
            %速度规划
            if %速度规划ok
                %生成轨迹
            else
                %紧急制动轨迹
            end
        end
    end
    if %无需换道或换道不ok且重规划中
        %速度规划
        if %速度规划ok
            %生成轨迹
        else
            %紧急制动轨迹
        end

    elseif %无需换道或换道不ok且需重规划
        %重规划原车道+ 速度规划---------------------------------
        %速度规划
    else 
        %速度规划
        if %速度规划ok
            %生成轨迹
        else
            %紧急制动轨迹
        end 
    end
end
%轨迹生成
