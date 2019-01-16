function z = integrndOnePoint(x, y, a) 
%integrndOnePoint 每一个（x,y）点对应的被积函数
z = f_a(a)*y*sin(x);%+x*cos(y);
end

function f_a = f_a(a)
    f_a = 10*a;
end

