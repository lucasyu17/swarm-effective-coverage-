function isDrawn = drawFigures(ResCell,X,Y,number_agent,diffs_mesh)
%drawFigures draws the related pictures
velocitiesTime = ResCell.velocitiesTime;
velos_abs = zeros(number_agent,size(velocitiesTime,2));
velos_x = zeros(number_agent,size(velocitiesTime,2));
velos_y = zeros(number_agent,size(velocitiesTime,2));
diffMeshTime = ResCell.diffMeshTime;
timeseries = ResCell.timeseries;
diffsTime = ResCell.diffsTime;
agentPosTime = ResCell.agentPosTime;

%% plot the norm of velocities
lgs = cell(1,number_agent);

for i_time = 1:size(velocitiesTime,2)
    velo_tmp = velocitiesTime{:,i_time};
    velos_abs(:,i_time) = sqrt(power(velo_tmp(:,1),2) + power(velo_tmp(:,2),2));
    velos_x(:,i_time) = velo_tmp(:,1);
    velos_y(:,i_time) = velo_tmp(:,2);
end
figure
[ha, pos] = tight_subplot(2,1,[.1 .055],[.05 .05],[.05 .05]);
% subplot(2,1,1)
axes(ha(1))
grid minor
hold on
ylabel('velocitiesX')
title('velocities in x direction')
for i_velo = 1:size(velos_abs,1)
    plot(velos_x(i_velo,:),'LineWidth',1.5)
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)

% subplot(2,1,2)
axes(ha(2))
grid minor
hold on
ylabel('velocitiesY')
title('velocities in y direction')
for i_velo = 1:size(velos_abs,1)
    plot(velos_y(i_velo,:),'LineWidth',1.5)%./velos_abs(i_velo,:))
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)
%% plot the err 
figure 
grid minor
hold on
ylabel('totalErr')
plot(timeseries,diffsTime,'LineWidth',1.5);

%% plot the errMesh at certain time steps
figure
[ha, pos] = tight_subplot(3,2,[.1 .055],[.05 .05],[.05 .05]);

% grid minor 
axes(ha(1))
mesh(X,Y,diffMeshTime{1})
view([90,90])
title('1/6 total time')

% tight_subplot(3,2,[.01 .03],[.1 .01],[.01 .01])
% grid minor 
axes(ha(2))
mesh(X,Y,diffMeshTime{2})
view([90,90])
title('2/6 total time')
% 
% subplot(3,2,[.01 .03],[.1 .01],[.01 .01])
% grid minor 
axes(ha(3))
mesh(X,Y,diffMeshTime{3})
view([90,90])
title('3/6 total time')
% 
% subplot(3,2,4)
% grid minor 
axes(ha(4))
mesh(X,Y,diffMeshTime{4})
view([90,90])
title('4/6 total time')
% 
% subplot(3,2,5)
% grid minor 
axes(ha(5))
mesh(X,Y,diffMeshTime{5})
view([90,90])
title('5/6 total time')
% 
% subplot(3,2,6)
% grid minor 
axes(ha(6))
mesh(X,Y,diffs_mesh)
view([90,90])
title('total time')
% 
% set(ha(1),'XTickLabel','1/6 total time');
% set(ha(2),'XTickLabel','2/6 total time');
% set(ha(3),'XTickLabel','3/6 total time');
% set(ha(4),'XTickLabel','4/6 total time');
% set(ha(5),'XTickLabel','5/6 total time');
% set(ha(6),'XTickLabel','6/6 total time');

isDrawn = true;

%% plot the positions evolution of uav1
positionsTime_x = zeros(number_agent,size(agentPosTime,2));
positionsTime_y = zeros(number_agent,size(agentPosTime,2));
for i_time = 1:size(agentPosTime,2)
    pos_tmp = agentPosTime{:,i_time};
    positionsTime_x(:,i_time) = pos_tmp(:,1);
    positionsTime_y(:,i_time) = pos_tmp(:,2);
end
figure
% subplot(2,1,1)
[ha, pos] = tight_subplot(2,1,[.1 .055],[.05 .05],[.05 .05]);
axes(ha(1))
grid minor 
hold on
ylabel('positionsX')
title('x position')
for i_pos = 1:size(positionsTime_x,1)
    plot(positionsTime_x(i_pos,:),'LineWidth',1.5)
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)
% subplot(2,1,2)
axes(ha(2))
grid minor 
hold on
ylabel('positionsY')
title('y position')
for i_pos = 1:size(positionsTime_y,1)
    plot(positionsTime_y(i_pos,:),'LineWidth',1.5)
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)
end

