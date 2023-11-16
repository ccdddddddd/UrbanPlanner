function node2expand=selectionBest(node)%输入：树，输出：叶节点索引
    successor=1;
    while nnz(node(successor).children)~=0
        successorCandi=nonzeros(node(successor).children); % 备选继承人nodeID数组
        rewardOfSuccessorCandi=zeros(1,length(successorCandi));
        for i=1:1:length(successorCandi)
            rewardOfSuccessorCandi(i)=node(successorCandi(i)).totalReward;
        end
        [~,bestOfSuccessorCandiIndex]=max(rewardOfSuccessorCandi); % 最优继承人在备选继承人nodeID数组中的索引
        % bestOfSuccessorCandi=successorCandi(bestOfSuccessorCandiIndex); % 最优继承人
        % epsilon=1/max(1,node(1).numVisits);
        % probOfSuccessorCandi=zeros(1,length(successorCandi)); % 最优继承人之外的继承人的继承概率
        % probOfSuccessorCandi(bestOfSuccessorCandiIndex)=1; % 最优继承人的继承概率
        % successor=randsrc(1,1,[successorCandi;probOfSuccessorCandi]); % 根据备选继承人nodeID数组和备选继承人继承概率数组，得到继承人的nodeID
        successor=successorCandi(bestOfSuccessorCandiIndex);
    end
    node2expand=successor;
end