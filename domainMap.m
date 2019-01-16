function [mapTargetsIndx,mapAgentsIndx] = domainMap(agentPos, targetPos, r_i)
%W_i calculate the sensory domain of the i_th agent
mapTargetsIndx = containers.Map(); % 与agent覆盖区域有交集的目标区域的点作为key值，与所对应的agents的序号形成的映射
mapAgentsIndx = containers.Map();  % 与目标区域点集内有交集的agent作为key的映射
cell_agents = cell(size(agentPos,1),1);
for i_target =1:size(targetPos,1)
    relate_agentPos = [];
    for i_agent =1:size(agentPos,1)
        if distanceSquare(agentPos(i_agent,:),targetPos(i_target,:)) <= r_i*r_i
            relate_agentPos = [relate_agentPos;i_agent];
            cell_agents{i_agent} = [cell_agents{i_agent},i_target];
        end
    end
    if ~isempty(relate_agentPos)
        mapTargetsIndx(num2str(i_target)) = relate_agentPos;
    end
end
for i_agent = 1:size(agentPos,1)
    if sum(cell_agents{i_agent}) ~=0
        mapAgentsIndx(num2str(i_agent)) = cell_agents{i_agent};
    end
end
end

function s = distanceSquare (Point1,Point2)
s = power(Point1(1,1)-Point2(1,1),2) + power(Point1(1,2)-Point2(1,2),2);
end

