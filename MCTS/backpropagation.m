function [node,gains]=backpropagation(leafNodeID,gainsSimulation,node,gamma)%输入：叶节点索引，叶节点模拟收益，树，折扣系数
    predecessor=leafNodeID;
    node(1).numVisits=node(1).numVisits+1;%更新根节点访问次数
    gainsSuccessor=gainsSimulation;%继承人的收益
    gains=-100000;
    while predecessor~=1
        node(predecessor).numVisits=node(predecessor).numVisits+1;%更新被继承人访问次数
%         gains=reward(node(node(predecessor).parent).state,node(predecessor).state)+gamma*gainsSuccessor;%被继承人的收益
%         gains=reward(node(node(predecessor).parent).state,node(predecessor).state,numOfVehicles)+gamma*gainsSuccessor;%被继承人的收益
        gains=node(predecessor).actionReward+gamma*gainsSuccessor;%被继承人的收益
        node(predecessor).totalReward=node(predecessor).totalReward+(gains-node(predecessor).totalReward)/node(predecessor).numVisits;%更新被继承人累计收益
        gainsSuccessor=gains;
        parent=node(predecessor).parent;
        if node(predecessor).isFullyExpanded==1 && sum(node(predecessor).children)==0
            predecessorIndex=find(node(parent).children==predecessor);
            node(parent).children(predecessorIndex)=0;
        end
        predecessor=parent;
    end
end