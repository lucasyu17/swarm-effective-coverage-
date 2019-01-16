function velocities  = e_greedyVelocities( agentPositions, velocities, r_communication, epsilon )
%e_greedyVelocities: give the agents e-possibility to discover the area
inrangeMap = inRange(agentPositions, r_communication);
rand_epsilon = rand(size(agentPositions,1),1);
for i_agent = 1:size(agentPositions,1)
    if ~isempty(inrangeMap(num2str(i_agent))) && rand_epsilon(i_agent) < epsilon %epsilon-greedy£ºdiscover
        near_indexs = inrangeMap(num2str(i_agent));
        velocities(i_agent) = velocities(i_agent) - ...
            2.*sum(velocities(near_indexs,:),1)/size(inrangeMap(num2str(i_agent)));%-...
            %(sum(agentPositions(near_indexs,:),1)./size(inrangeMap(num2str(i_agent)))-agentPositions(i_agent,:));
    end
end
end

function inrangeMap = inRange(agentPositions, r_communication)
inrangeMap = containers.Map();
dists_square = distFromVecs(agentPositions);
for i_agent = 1:size(agentPositions,1)
    positives = 0 <  dists_square(i_agent,:);
    inrangeMap(num2str(i_agent)) = find(dists_square(i_agent,positives) < r_communication*r_communication);
end
end

function dists_square = distFromVecs(agentPositions)
dists_square = zeros(size(agentPositions,1),size(agentPositions,1));
for i_agent = 1:size(agentPositions,1)
    temp = (agentPositions - agentPositions(i_agent,:))'...
                            .* (agentPositions - agentPositions(i_agent,:))';
    dists_square(i_agent,:) = temp(1,:) + temp(2,:);
end
end