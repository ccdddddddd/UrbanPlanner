function successor=selection(node,predecessor,CalibrationVars)%输入：树，被继承人，输出：继承人节点索引
    epsilonSwitch=CalibrationVars.epsilonSwitch;
    successorCandi=nonzeros(node(predecessor).children); % 备选继承人nodeID数组
%     rewardOfSuccessorCandi=cat(2,node(successorCandi).totalReward);
    rewardOfSuccessorCandi=zeros(1,length(successorCandi));
    for i=1:1:length(successorCandi)
        rewardOfSuccessorCandi(i)=node(successorCandi(i)).totalReward;
    end
    [~,bestOfSuccessorCandiIndex]=max(rewardOfSuccessorCandi); % 最优继承人在备选继承人nodeID数组中的索引
    epsilon=1/max(1,node(1).numVisits);
    if epsilonSwitch==3
        epsilon=(epsilon).^(1/4);
    elseif epsilonSwitch==2
        epsilon=sqrt(epsilon);
    elseif epsilonSwitch==4
        epsilon=0.1;
    elseif epsilonSwitch==5
        epsilon=0.7;
    end
    numOfChildren=length(successorCandi);
    probOfSuccessorCandi=zeros(1,numOfChildren)+epsilon/numOfChildren; % 最优继承人之外的继承人的继承概率
    probOfSuccessorCandi(bestOfSuccessorCandiIndex)=epsilon/numOfChildren+1-epsilon; % 最优继承人的继承概率
    % successor=randsrc(1,1,[successorCandi;probOfSuccessorCandi]); % 根据备选继承人nodeID数组和备选继承人继承概率数组，得到继承人的nodeID
    % probOfSuccessorCandi=[0.1,0.2,0.3,0.4]; 
    upperLimitOfProb=cumsum(probOfSuccessorCandi);
    randDouble=rand();
    successorNum=find(upperLimitOfProb>randDouble,1);
    successor=successorCandi(successorNum);
    successor=successor(1);
end