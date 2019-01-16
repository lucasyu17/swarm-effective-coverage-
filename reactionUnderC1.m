function [velocities,min_dist,MiniDistPts] = reactionUnderC1( agent_positions,diffs_mesh,X,Y,range_linear,k_linear )

[MiniDistPts,min_dist] = switchToLinearControl(agent_positions,diffs_mesh,X,Y,range_linear);
velocities = -k_linear .* (agent_positions - MiniDistPts);

end

function [MiniDistPts,min_dist] = switchToLinearControl(agentPoses,diffs_mesh,X,Y,range_linear)
MiniDistPts = zeros(size(agentPoses,1),2);
min_dist = zeros(size(agentPoses,1),1);
epsilon = 0.3;
rand_epsilon = rand(size(agentPoses,1),1);
for i_zeroAgent = 1:size(agentPoses,1)
    agent_pos = agentPoses(i_zeroAgent,:);
    dists = distanceSquareMesh (agent_pos,X,Y);
    dists = dists(:);
    X = X(:);
    Y = Y(:);
    diffs_mesh = diffs_mesh(:);
%     haveErr = find(diffs_mesh>1);
    min_dist(i_zeroAgent,1) = max(dists);
    min_indx = nan;
    max_dist(i_zeroAgent,1) = min(dists);
    max_indx = nan;
%     if(rand_epsilon(i_zeroAgent))>epsilon
    for i_err = 1:size(diffs_mesh)
        if dists(i_err)< min_dist(i_zeroAgent,1) && dists(i_err)> range_linear*range_linear*5 && diffs_mesh(i_err)>0
            min_dist(i_zeroAgent,1) = dists(i_err);
            min_indx = i_err;
        end
    end
    MiniDistPts(i_zeroAgent,:) = [X(min_indx),Y(min_indx)];
%     else
%         for i_err = 1:size(diffs_mesh)
%             if dists(i_err)> max_dist(i_zeroAgent,1) && diffs_mesh(i_err)>0
%                 max_dist(i_zeroAgent,1) = dists(i_err);
%                 max_indx = i_err;
%             end
%         end
%         MiniDistPts(i_zeroAgent,:) = [X(max_indx),Y(max_indx)];
%     end
end
end


function s = distanceSquareMesh (Point1,X,Y)
s = power(Point1(1,1)-X,2)+power(Point1(1,2)-Y,2);

end
