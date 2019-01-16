function agentsInitPositions = agents_init(nbAgents, sizeOfField, senseRanges)
%agents_init give the init positions of all agents: in the case that all 
%the agents begin with entering the field, meaning that they intersect with
%the boundaries.

agentsInitPositions = zeros(nbAgents,2);
xBegin = -sizeOfField(1)/2;
xEnd = sizeOfField(1)/2;
yBegin = -sizeOfField(2)/2;
yEnd = sizeOfField(2)/2;

for i = 1:nbAgents
    whichSizeToBegin = rand(4,1); %up,down,left,right
    whichSizeToBegin = find((whichSizeToBegin>=max(whichSizeToBegin)));
    switch whichSizeToBegin
        case 1 %up side
            agentsInitPositions(i,1) = xBegin + (xEnd-xBegin)*rand(1,1);
            agentsInitPositions(i,2) = yEnd - senseRanges(i)*rand(1,1);
        case 2 %down side
            agentsInitPositions(i,1) = xBegin + (xEnd-xBegin)*rand(1,1);
            agentsInitPositions(i,2) = yBegin + senseRanges(i)*rand(1,1);
        case 3 %left side
            agentsInitPositions(i,1) = xBegin + senseRanges(i)*rand(1,1);
            agentsInitPositions(i,2) = yBegin + (yEnd-yBegin)*rand(1,1);
        case 4 %right side
            agentsInitPositions(i,1) = xEnd - senseRanges(i)*rand(1,1);
            agentsInitPositions(i,2) = yBegin + (yEnd-yBegin)*rand(1,1);
    end
end

end

