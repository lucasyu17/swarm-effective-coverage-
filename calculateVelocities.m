function velocities = calculateVelocities( diffs_mesh, k_cov,k_col,k_linear, agentPositions, MaxValue, range, X,Y,x,y)
%integr 计算所有agent的速度。各变量维度：
%ks[nb_agents*1]，derivs_penalty[nb_target*1],agents[nb_agents*1],targets[nb_target*1]
velocities = zeros(size(agentPositions,1),2);

diffs_mesh = diffCurTargetMesh(diffs_mesh, agentPositions, X,Y, MaxValue, range);
% diff_Integr = trapz(y,trapz(x,diffs_mesh,2));
derivs_penalty = penalty_functionMesh(2,diffs_mesh); %penalty函数的导数值，带入q_i,q_tild点。

for i_agent =1:size(agentPositions,1)
    if ~isnan(agentPositions(i_agent,1))
        k_i = k_cov(i_agent,1);

        [posDiff_x,posDiff_y] = diffVecMesh(agentPositions(i_agent,:),X,Y);
        [isDistsInRange,dervAs] = derivAFromMesh( agentPositions(i_agent,:), X, Y, MaxValue, range );

        elementToBeIntegr_x = isDistsInRange .* posDiff_x .* derivs_penalty .* dervAs .* k_i;
        elementToBeIntegr_y = isDistsInRange .* posDiff_y .* derivs_penalty .* dervAs .* k_i;
        Integr_vx = trapz(y,trapz(x,elementToBeIntegr_x,2));
        Integr_vy = trapz(y,trapz(x,elementToBeIntegr_y,2));
        velocities(i_agent,:) = [Integr_vx,Integr_vy]; 
    
    else
        continue;
    end
end

% collision avoidance velocity 
velocities = CollisionAvoid( velocities,k_col,agentPositions,range*1.5,range*1.2 );
% linear velocites
% veloSum = sum(abs(velocities),2);
% ZeroVelocities = find(veloSum<1e-3);
% if ~isempty(ZeroVelocities)
%     [linear_velocities,~] = reactionUnderC1( agentPositions(ZeroVelocities,:),diffs_mesh,X,Y,range,k_linear );
%     velocities(ZeroVelocities,:) = linear_velocities;
% end
end

function [isDistsInRange,dervAs] = derivAFromMesh(agent,X,Y,MaxValue,range)
%X,Y必须是相同的分辨率
dists = distanceSquareMesh(agent,X,Y);
isDistsInRange = dists<=range*range;
dervAs = isDistsInRange.*2.*MaxValue./power(range,4).*(dists-range*range);
end

function s = distanceSquareMesh (Point1,X,Y)
%X与Y是相同维度，返回值：每一个对应位置(x,y)上与Point1的距离值平方
s = power(Point1(1,1)-X,2)+power(Point1(1,2)-Y,2);
end

function  [resX,resY] = diffVecMesh(Point,X,Y)
%返回值：每一个点的差值向量的x,y分量
resX = Point(1)-X;
resY = Point(2)-Y;
end
