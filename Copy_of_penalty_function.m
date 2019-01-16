function derivs_penalty = penalty_function( degree,diffs )
%penalty_function cost function of the coverage task
derivs_penalty = [];
for i_diff =1:size(diffs,1)
    [ ~,deriv_val ] = penalty_functionOnOneVal( degree,diffs(i_diff) );
    derivs_penalty = [derivs_penalty;deriv_val];
end
end

function [ func_val,deriv_val ] = penalty_functionOnOneVal( degree,x )
%penalty_function cost function of the coverage task
if x<=0
    func_val = 0;
    deriv_val = 0;
else
    func_val = power(x,degree);
    deriv_val = degree*power(x,degree - 1);
end

end