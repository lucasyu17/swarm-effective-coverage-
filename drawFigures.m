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
subplot(2,1,1)
grid minor
hold on
ylabel('velocitiesX')
for i_velo = 1:size(velos_abs,1)
    plot(velos_x(i_velo,:))
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)

subplot(2,1,2)
grid minor
hold on
ylabel('velocitiesY')
for i_velo = 1:size(velos_abs,1)
    plot(velos_y(i_velo,:))%./velos_abs(i_velo,:))
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
plot(timeseries,diffsTime);

%% plot the errMesh at certain time steps
figure
subplot(3,2,1)
grid minor 
mesh(X,Y,diffMeshTime{1})
view([90,90])

subplot(3,2,2)
grid minor 
mesh(X,Y,diffMeshTime{2})
view([90,90])

subplot(3,2,3)
grid minor 
mesh(X,Y,diffMeshTime{3})
view([90,90])

subplot(3,2,4)
grid minor 
mesh(X,Y,diffMeshTime{4})
view([90,90])

subplot(3,2,5)
grid minor 
mesh(X,Y,diffMeshTime{5})
view([90,90])

subplot(3,2,6)
grid minor 
mesh(X,Y,diffs_mesh)
view([90,90])

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
subplot(2,1,1)
grid minor 
hold on
ylabel('positionsX')
for i_pos = 1:size(positionsTime_x,1)
    plot(positionsTime_x(i_pos,:))
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)
subplot(2,1,2)
grid minor 
hold on
ylabel('positionsY')
for i_pos = 1:size(positionsTime_y,1)
    plot(positionsTime_y(i_pos,:))
    hold on
end
for i_agent = 1:number_agent
    lgs{1,i_agent} = num2str(i_agent);
    hold on
end
legend(lgs)
end

