function diffs_mesh = diffCurTargetMesh( diffs_mesh, agents,X,Y,MaxValue,r )
% [X,Y] = meshgrid(targets(:,1),targets(:,2));

for i_agent =1:size(agents,1)
    [eff,~] = A_circularMesh(agents(i_agent,:),X,Y,MaxValue,r);
    diffs_mesh = diffs_mesh - eff;
end
diffs_mesh = diffs_mesh .* (diffs_mesh>0);

end