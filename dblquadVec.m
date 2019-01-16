function integrAns = dblquadVec(x_min,x_max,y_min,y_max,a )
%dblquadVec 多维度的积分
%x_min: n*1
integrAns = [];
for i_param = 1:size(x_min,1)
    integrAns = [integrAns;dblquad(@integrndOnePoint,x_min(i_param,1),x_max(i_param,1),y_min(i_param,1),y_max(i_param,1),[],[],a)];
end
end


function z = integrndOnePoint(x, y, a) 
%integrndOnePoint 每一个（x,y）点对应的被积函数
z = f_a(a)*y*sin(x)+x*cos(y);
end

function f_a = f_a(a)
    f_a = 10*a;
end


% function integrAns = dblquadVec(x_min,x_max,y_min,y_max )
% %dblquadVec 多维度的积分
% %x_min: (nb_agents,1)
% integrAns = [];
% for i_param = 1:size(x_min,1)
%     integrAns = [integrAns;dblquad(@f_x,x_min(i_param,1),x_max(i_param,1),y_min(i_param,1),y_max(i_param,1))];
% end
% end

% function z = f_x(x,y,agents, MaxValue, range)
%     MaxValue = 1;
%     range = 0.2;
%     target = [x,y];
%     agents = [1,0];
%     a = derivAllTargets( agents(1,:), target, MaxValue, range);
%     z = x*a;
% end




