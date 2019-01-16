function velocities = calculateVelocities( ks, derivs_penalty, agents, targets, MaxValue, range, targetSize)
%integr 计算所有agent的速度。各变量维度：
%ks[nb_agents*1]，derivs_penalty[nb_target*1],agents[nb_agents*1],targets[nb_target*1]
velocities = [];

for i_agent =1:size(agents,1)
    k_i = ks(i_agent,1);
%     pos_diff = agents(i_agent,:)-targets;
%     dervAs = derivAllTargets( agents(i_agent,:), targets, MaxValue, range );
%     elementToBeIntegr = pos_diff.*derivs_penalty.*dervAs*k_i;
%     velocities = [velocities;sum(elementToBeIntegr,1)]; 
    integrAns = k_i.*dblquadVec1(-targetSize(1,1)/2,targetSize(1,1)/2,-targetSize(1,2)/2,targetSize(1,2)/2, agents(i_agent,:),MaxValue,range);
    velocities = [velocities;integrAns];
end
end

function integrAns = dblquadVec1(x_min,x_max,y_min,y_max, agent,MaxValue,range)
%dblquadVec 多维度的积分
%x_min: (nb_agents,1)
integrAns = [];
for i_param = 1:size(x_min,1)
    integrAns_x = dblquad(@f_x,x_min(i_param,1),x_max(i_param,1),y_min(i_param,1),y_max(i_param,1),[],[], agent,MaxValue,range);
    integrAns_y = dblquad(@f_y,x_min(i_param,1),x_max(i_param,1),y_min(i_param,1),y_max(i_param,1),[],[], agent,MaxValue,range);
    integrAns = [integrAns;integrAns_x,integrAns_y];
end
end

function z = f_x(x,y, agent,MaxValue,range)
    target = [x,y];
    z = x*derivAllTargets( agent,target,MaxValue,range);
end

function z = f_y(x,y, agent,MaxValue,range)
    target = [x,y];
    z = y*derivAllTargets( agent,target,MaxValue,range);
end

function dervAs = derivAllTargets( agent, targets, MaxValue, range )
%derivAllTargets gives the derivative of A in function of s, to all the
%targets
dists = distanceSquare(agent,targets);
dervAs = [];
isDistsInRange = dists<=range*range;
dervAs = isDistsInRange*2*MaxValue/power(range,4).*(dists-range*range);
end

function s = distanceSquare (Point1,Point2s)
s = [];
for i_pt2 = 1:size(Point2s,1)
    Point2 = Point2s(i_pt2,:);
    s = [s;power(Point1(1,1)-Point2(1,1),2)+...
            power(Point1(1,2)-Point2(1,2),2)];
end
end