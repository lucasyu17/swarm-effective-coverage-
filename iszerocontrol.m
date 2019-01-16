function [isZeroControl, ZeroVelPositions, linearVelocities,min_dist] = iszerocontrol(agentPos,velocities,diffs_mesh,X,Y,linear_k,range)
% if the agents enter into a situation where velocities are zeros while err
% are not, it should be changed into a linear control law.
velos = sqrt(velocities(:,1).*velocities(:,1) + velocities(:,2).*velocities(:,2));
velos = velos.*agentPos(:,1)./agentPos(:,1);

posToWipeOut = [];
ZeroVelPositions = find(velos <= 1e-4);
for i_zepos = 1:size(ZeroVelPositions)
    agent_tmp = agentPos(ZeroVelPositions(i_zepos),:);
    dists = distanceSquareMesh (agent_tmp,X,Y);
    dists = dists(:);
    diffs_meshfake = diffs_mesh(:);
    isinrange = dists < range;
    diffs_meshfake = diffs_meshfake.*isinrange;
    if sum(diffs_meshfake)>=1e-5
        posToWipeOut = [posToWipeOut,i_zepos];
    end
end
ZeroVelPositions(posToWipeOut,:) = [];
linearVelocities = zeros(size(ZeroVelPositions,1),2);
isZeroControl = ~isempty(ZeroVelPositions);
min_dist = zeros(size(ZeroVelPositions,1),1);
if isZeroControl
    [MiniDistPts,min_dist] = switchToLinearControl(agentPos,ZeroVelPositions,diffs_mesh,X,Y,range);
    linearVelocities = -linear_k .* (agentPos(ZeroVelPositions,:) - MiniDistPts);
end
end

function [MiniDistPts,min_dist] = switchToLinearControl(agentPoses,zeroCtrlAgents,diffs_mesh,X,Y,range)
MiniDistPts = zeros(size(zeroCtrlAgents,1),2);
min_dist = zeros(size(zeroCtrlAgents,1),1);
for i_zeroAgent = 1:size(zeroCtrlAgents,1)
    agent_pos = agentPoses(zeroCtrlAgents(i_zeroAgent,1),:);
    dists = distanceSquareMesh (agent_pos,X,Y);
    dists = dists(:);
    X = X(:);
    Y = Y(:);
    diffs_mesh = diffs_mesh(:);
    haveErr = find(diffs_mesh>1);
    min_dist(i_zeroAgent,1) = max(dists);
    min_indx = 1;
    for i_err = 1:size(haveErr)
        if dists(i_err)<=min_dist(i_zeroAgent,1) && dists(i_err)>=range
            min_dist(i_zeroAgent,1) = dists(i_err);
            min_indx = i_err;
        end
    end
    MiniDistPts(i_zeroAgent,:) = [X(min_indx),Y(min_indx)];
end
end

function s = distanceSquareMesh (Point1,X,Y)
%X与Y是相同维度，返回值：每一个对应位置(x,y)上与Point1的距离值平方
s = power(Point1(1,1)-X,2)+power(Point1(1,2)-Y,2);
end

function unitVec = unitVecFromTwoPts(Pt1,Pt2)
diffVec =  Pt1 - Pt2;
unitVec = diffVec./(sqrt(sum(diffVec.*diffVec)));
end
function len = length_vec(vec)
len = sqrt(sum(vec.*vec));
end
