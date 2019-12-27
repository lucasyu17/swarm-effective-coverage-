trj_pos = Results.agentPosTime;
len = size(Results.agentPosTime,2)
trj_pos = zeros(len, number_agent*2);
relative_pos = zeros(len, number_agent*2);
for i=1:len
    position = Results.agentPosTime{i}
    for i_agent=1:number_agent
        trj_pos(i,(i_agent-1)*2+1) = position(i_agent,1);
        trj_pos(i,i_agent*2) = position(i_agent,2);
    end
    
end
trj_pos;
relative_pos(2:end,:) = trj_pos(2:end,:) - trj_pos(1:end-1,:);

txt_file = fopen('trajectories.txt','wt');
fprintf(txt_file,'scan 1\n');
fprintf(txt_file,'battery_check 50\n');
fprintf(txt_file,'correct_ip\n');
fprintf(txt_file,'1=0TQZG9SED004UG\n');
% fprintf(txt_file,'2=0TQZG9SED004R1\n');
% fprintf(txt_file,'3=0TQZG9RED0007Z\n');
% fprintf(txt_file,'4=0TQZG9RED0000D\n');
% fprintf(txt_file,'5=0TQZG9RED001DU\n');

fprintf(txt_file,'*>takeoff\n');
relative_pos(1:end-1,1:end) = alpha .* relative_pos(1:end-1,1:end) + (1-alpha) .* relative_pos(2:end,1:end);
len_trj=size(relative_pos(:,1));
for i_trj=1:len_trj/2
    x = ceil((relative_pos(i_trj,1)*1000));
    y = ceil((relative_pos(i_trj,2)));
    if (0<x && x<20)
        x = 20;
    end
    if (-20<x && x<0)
        x = 20;
    end
    if (0<y && y<20)
        y = 20;
    end
    if (-20<y && y<0)
        y = 20;
    end
    fprintf(txt_file,'*>go  %d  %d  0 30 \n',x,y);
    fprintf(txt_file,'delay 10\n');
end
fprintf(txt_file,'*>land\n');

figure
alpha = 0.5;

plot(relative_pos(:,3:4))
