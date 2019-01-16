function [effectivity,deriv_s] = A_circularMesh( point, X,Y, MaxValue, range )
%circular coverage function
% [X,Y] = meshgrid(targets(:,1),targets(:,2));
if isnan(point(1,1))
    effectivity = 0;
    deriv_s = 0;
else
    s = distanceSquareMesh(point,X,Y); %mesh
    isSmallerThanRange = s< range*range; %mesh

    effectivity = isSmallerThanRange.*MaxValue./power(range,4).*power((s - range*range),2); %mesh
    deriv_s = isSmallerThanRange.*2.*MaxValue./power(range,4).*(s - range*range); %mesh
end
end

function s = distanceSquareMesh (Point1,X,Y)
s = (Point1(1,1)-X) .* (Point1(1,1)-X) + (Point1(1,2)-Y) .* (Point1(1,2)-Y);
end
