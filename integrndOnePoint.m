function z = integrndOnePoint(x, y, a) 
%integrndOnePoint ÿһ����x,y�����Ӧ�ı�������
z = f_a(a)*y*sin(x);%+x*cos(y);
end

function f_a = f_a(a)
    f_a = 10*a;
end

