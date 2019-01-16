clear all
close all
clc
%% initializations
MaxValue = 1;
q = [0,0];
r = 0.2;

resolution_field = 4;
number_agent = 16;
targetFieldSize = [8,8];
agentMeshSize = [-r*1.2,r*1.2];
resolutionMesh = 80;
agentInit = agents_init(number_agent,targetFieldSize);
targetinit = targets_init(targetFieldSize,resolution_field);
agentMesh = targets_init(agentMeshSize,resolutionMesh);

%%
[X,Y] = meshgrid(targetinit(:,1),targetinit(:,2));
Effects_mesh = zeros(resolutionMesh,resolutionMesh);
for i = 1:resolutionMesh
    for j = 1:resolutionMesh
        q_tild = [agentMesh(i,1),agentMesh(j,2)];
        Effects_mesh(i,j) = A_circular(q,q_tild,MaxValue,r);
    end
end
total_capacity = sum(sum(Effects_mesh));

%% define the target coverage C_star
C_star = total_capacity*0.5;
ks = 0.08.*ones(size(agentInit,1),1);

%% integrade
x_min = pi.*ones(5,1);
x_max = 2*pi.*ones(5,1);
y_min = zeros(5,1);
y_max = pi.*ones(5,1);
Q = dblquadVec(x_min,x_max,y_min,y_max,10);

%% calculate the difference between current and target coverage
dt = 0.2;
timeseries = [];
diffsTime = [];
velocitiesTime = [];
for time = 0:dt:100
    timeseries = [timeseries,time];
    diffs = diffCurTarget(C_star,agentInit,targetinit,MaxValue,r);
    derivs_penalty = penalty_function(2,diffs);
    velocities = calculateVelocities(ks,derivs_penalty,agentInit,targetinit,MaxValue,r,targetFieldSize);
    agentInit = agentInit + velocities.*dt;
    velocitiesTime = [velocitiesTime,velocities];
    diffsTime = [diffsTime,diffs];
end


%% show the initial situation and velocity direction 
figure 
hold on
grid on
mesh = meshgrid(targetinit(:,1),targetinit(:,2));
for indx_i = 1:size(targetinit,1)
    for indx_j = 1:size(targetinit,1)
        plot(targetinit(indx_i,1),targetinit(indx_j,2),'o')
        hold on
    end
end
for indx_i = 1:size(agentInit,1)
    plot(agentInit(indx_i,1),agentInit(indx_i,2),'*')
    hold on
end
for i =1:size(agentInit,1)
    velocities(i,1) = velocities(i,1);%/sqrt(power(velocities(i,1),2)+power(velocities(i,2),2));
    velocities(i,2) = velocities(i,2);%/sqrt(power(velocities(i,1),2)+power(velocities(i,2),2));
    quiver(agentInit(i,1),agentInit(i,2),velocities(i,1),velocities(i,2))
end




