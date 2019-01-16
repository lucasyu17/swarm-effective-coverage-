function [ velocities ] = CollisionAvoid( velocities,k_col,agentPositions,R,r )

% collision avoidance velocity 

agent_x = agentPositions(:,1);
agent_y = agentPositions(:,2);

diff_dist_x = agent_x'-agent_x;
diff_dist_x_sq = diff_dist_x.*diff_dist_x;

diff_dist_y = agent_y'-agent_y;
diff_dist_y_sq = diff_dist_y.*diff_dist_y;

diff_dist = sqrt(diff_dist_x_sq + diff_dist_y_sq);
diff_dist_sq = diff_dist.*diff_dist;

% distance in the range of [r,R]
isInRange = (diff_dist<R) .* (diff_dist>r);

coeff = 4*(R*R - r*r) .* (diff_dist_sq - R*R)./power((diff_dist_sq - r*r),3) .* isInRange;
v_x = sum(coeff.*diff_dist_x)';
v_y = sum(coeff.*diff_dist_y)';
v_col = -k_col.*[v_x,v_y];
velocities = velocities + v_col;

end

