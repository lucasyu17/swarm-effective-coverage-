function [effectivity,deriv_s] = A_circular( q, targets, MaxValue, range )
%定义了instantaneous coverage function D*Q-->R+

s = distanceSquare(q,targets);
isSmallerThanRange = s<=range*range;

effectivity = isSmallerThanRange.*MaxValue/power(range,4).*power((s - power(range,2)),2);
deriv_s = isSmallerThanRange.*2.*MaxValue/power(range,4).*(s - range*range);

end

function s = distanceSquare (Point1,Point2s)
s = zeros(size(Point2s));
for i_pt2 = 1:size(Point2s,1)
    Point2 = Point2s(i_pt2,:);
    s(i_pt2) = power(Point1(1,1)-Point2(1,1),2)+...
            power(Point1(1,2)-Point2(1,2),2);
end
end
