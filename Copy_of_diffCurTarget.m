function diffs = diffCurTarget( C_star,agents,targets,MaxValue,r )
%diffCurTarget Summary of this function goes here

% [mapTarIndx,mapAgentIndx] = domainMap(agents,targets,r);
diffs = ones(size(targets,1),1).*C_star;

% if isincell(mapTarIndx.keys,i_target)
%     try
%         agentstmp = mapTarIndx(num2str(i_target));
%     catch
%         disp(i_target);
%     end
for i_agent =1:size(agents,1)
    [eff,~] = A_circularMesh(agents(i_agent,:),targets,MaxValue,r);
    diffs = diffs-eff;
end
for i_target = 1:size(targets,1)
    if diffs(i_target,1)<=0
            diffs(i_target,1) = 0;
    end
end
% end

end



function Resagents = findAgentsFromMap(mapTargetIndx,key,agentsPos)
if 0
else
    agents = mapTargetIndx(key);
    for i_agent = agents(1:end)
        Resagents = [Resagents;agentsPos(i_agent,:)];
    end
end
end

function isin = isincell(cell,number)
    nbstr = num2str(number);
    isin = false;
    for i = 1:size(cell,2)
        if strcmp(cell{i},nbstr)
            isin = true;
            return
        end
    end
    isin = false;
end