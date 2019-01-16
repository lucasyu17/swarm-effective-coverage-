function [mapTargetsIndx,mapAgentsIndx] = domainMap(agentPos, targetPos, r_i)
%W_i calculate the sensory domain of the i_th agent
mapTargetsIndx = containers.Map(); % ��agent���������н�����Ŀ������ĵ���Ϊkeyֵ��������Ӧ��agents������γɵ�ӳ��
mapAgentsIndx = containers.Map();  % ��Ŀ������㼯���н�����agent��Ϊkey��ӳ��
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

