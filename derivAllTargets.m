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

