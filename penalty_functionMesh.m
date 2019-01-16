function derPenalty = penalty_functionMesh( degree,diffsMesh )
%penalty_function cost function of the coverage task
% isPositive = diffsMesh>0;
% penalty = isPositive.*power(diffsMesh,degree);
derPenalty = (diffsMesh>0) .* degree .* power(diffsMesh,degree - 1);
end