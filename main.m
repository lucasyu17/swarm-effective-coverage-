clear all
close all
clc
%% initializations
MaxValue = 1;
r = 0.5;
r_communication = 3 * r;
epsilon = 0.3;

Results = {};
Results.timeseries = 0:dt:endTime;
Results.init_positions = zeros(number_agent,2);

resolution_field = 4;
number_agent = 5;
targetFieldSize = [8,8];
senseRanges = r.*ones(number_agent,1);
agentPositions = agents_init(number_agent,targetFieldSize,senseRanges);
agentInitPosition = agentPositions;
Results.init_positions = agentInitPosition;
targetinit = targets_init(targetFieldSize,resolution_field);
% velocities_buffer = zeros(number_agent,2);

%% for calculate the total coverage capacity of an agent
agentMeshSize = [r*1.2,r*1.2];
resolutionMesh = 100;
agent = [0,0];
agentGrids = targets_init(agentMeshSize,resolutionMesh);
x = agentGrids(:,1);
y = agentGrids(:,2);
[X_grid,Y_grid] = meshgrid(x,y);
Effects_mesh = A_circularMesh(agent,X_grid,Y_grid,MaxValue,r);
total_capacity = trapz(agentGrids(:,2),trapz(agentGrids(:,1),Effects_mesh,2));

%% define the target coverage C_star
C_star = total_capacity*10;
% coverage guidance
k_cov = 0.23*ones(number_agent,1);
% collision avoidance
k_col = 0.008*ones(number_agent,1); 
%deadlock linear coef
k_linear = 0.07;
%% calculate the difference between current and target coverage
dt = 0.5;
endTime = 700;

Results.diffsTime = zeros(1,endTime/dt);
Results.agentPosTime = cell(1,endTime/dt);
Results.diffMeshTime = cell(1,6);
resolution = 300;

Results.velocitiesTime = cell(1,endTime/dt);
i_time =1;
diffs_mesh = C_star;% * targetFieldSize(1,1) * targetFieldSize(1,2);
diffs = C_star * targetFieldSize(1,1) * targetFieldSize(1,2);
diffs_buffer = C_star * targetFieldSize(1,1) * targetFieldSize(1,2);
numberOfZeroControl = 0;
errChangeRateThres = 1e-4;

meshTargetField = targets_init(targetFieldSize,resolution);
x = meshTargetField(:,1);
y = meshTargetField(:,2);
[X,Y] = meshgrid(x,y);

% algorithm and renew of positions and errs
timeshares = floor(endTime / 6);
isDeadLock = false;

trj_positions = zeros(endTime/dt+1,number_agent*2);
trj_velocities = zeros(endTime/dt+1,number_agent*2);
%%
figure
title('覆盖任务初始�?')
line([-4,-4],[-4,4], 'LineWidth', 1.5, 'Color', 'k')
hold on
line([-4,4],[4,4], 'LineWidth', 1.5, 'Color', 'k')
line([4,4],[4,-4], 'LineWidth', 1.5, 'Color', 'k')
line([4,-4],[-4,-4], 'LineWidth', 1.5, 'Color', 'k')

% for i_agent = 1:number_agent
%     t=deg2rad(0:360);
%     x_circle=(r*cos(t)+agentPositions(i_agent,1));
%     y_circle=(r*sin(t)+agentPositions(i_agent,2));
% 
%     plot(x_circle,y_circle);hold on;
%     fill(x_circle,y_circle,'b')
% %         plot(agentPositions(i_agent,1),agentPositions(i_agent,2),'o','linewidth',2.0,'fill','b')
% %         lgs{1,i_agent} = num2str(i_agent);
%     hold on
% end
%%
figure
count = 0;
agent_last_pos = agentPositions;
for time = 0:dt:endTime
    count = count + 1
%     if count == 2
%         keyboard
%     end
    if time > endTime/2
        k_cov = 0.5*ones(number_agent,1);
    end
%     hold off
%     lgs = cell(1,number_agent);
    for i_agent = 1:number_agent
%         trj_positions(count,(i_agent-1)*2+1) = agentPositions(i_agent,1);
%         trj_positions(count,i_agent*2) = agentPositions(i_agent,2));
        t=deg2rad(0:360);
        x_circle=(r*cos(t)+agentPositions(i_agent,1));
        y_circle=(r*sin(t)+agentPositions(i_agent,2));
        
        plot(x_circle,y_circle);hold on;
        fill(x_circle,y_circle,'b','edgecolor','k','Linewidth',2.0);hold on
        
        u=deg2rad(0:360);
        x_circle_last = (r*cos(u)+agent_last_pos(i_agent,1));
        y_circle_last = (r*sin(u)+agent_last_pos(i_agent,2)); 
        
        plot(x_circle_last,y_circle_last);hold on;
        fill(x_circle_last,y_circle_last,'b','edgecolor','b'); hold on
        
%         plot(agentPositions(i_agent,1),agentPositions(i_agent,2),'o','linewidth',2.0,'fill','b')
%         lgs{1,i_agent} = num2str(i_agent);
        hold on
    end
    
    agent_last_pos = agentPositions;
%     legend(lgs)
    xlim([-targetFieldSize(1)/2-0.2,targetFieldSize(2)/2+0.2])
    ylim([-targetFieldSize(1)/2-0.2,targetFieldSize(2)/2+0.2])
%     text(-targetFieldSize(1)/2-0.5,targetFieldSize(2)/2+0.5,{'t: '; num2str(time)})
%     text(-targetFieldSize(1)/2+1,targetFieldSize(2)/2+0.5,{'err: '; num2str(diffs)})
%     hold on
%     grid on
    pause(0.005)
    
    if diffs/(C_star * targetFieldSize(1,1) * targetFieldSize(1,2)) <= 1e-6 % coverage task completed
        Results.diffsTime(1,i_time) = diffs;
        break;
    else
        if isDeadLock % Condition 1
        	if sum(abs(min_dist) < r*r*0.5) < number_agent % not all arrived
                aa = 1;
                velocities = -k_linear .* (agentPositions - MiniDistPts) ./ sqrt(sum((agentPositions - MiniDistPts).*(agentPositions - MiniDistPts), 2));
%                 indx_nonzero = find(sum(abs(velocities),2)>0);
%                 for i_indx = indx_nonzero(1:end)
%                     velocities(i_indx,1) = max(velocities(i_indx,1),0.01);
%                     velocities(i_indx,2) = max(velocities(i_indx,2),0.01);
%                 end 
                velocities = CollisionAvoid( velocities, k_col/2, agentPositions, r*2.5, r*1.1 );
                velocities = min(max(velocities,-0.2/dt),0.2/dt);
                agentPositions = agentPositions + velocities.*dt;
                agentPositions(:,1) = min(max(agentPositions(:,1),-targetFieldSize(1,1)/2),targetFieldSize(1,1)/2);
                agentPositions(:,2) = min(max(agentPositions(:,2),-targetFieldSize(1,2)/2),targetFieldSize(1,2)/2);
                min_dist = power(agentPositions(:,1) - MiniDistPts(:,1),2) + power(agentPositions(:,2) - MiniDistPts(:,2),2);
                Results.velocitiesTime{i_time} = velocities;
                diffs_mesh = diffCurTargetMesh(diffs_mesh, agentPositions, X,Y, MaxValue, r);
                diffs = trapz(y,trapz(x,diffs_mesh,2));
                diffs_buffer = diffs;
                Results.diffsTime(1,i_time) = diffs;
                Results.agentPosTime{i_time} = agentPositions;
                if ~isempty(find([timeshares,timeshares*2,timeshares*3,timeshares*4,timeshares*5] == i_time)) || i_time == endTime
                    Results.diffMeshTime{find([timeshares,timeshares*2,timeshares*3,timeshares*4,timeshares*5,endTime] == i_time)} = diffs_mesh;
                end
                i_time = i_time+1;
                continue
            else
                isDeadLock = false;
            end
        end
        if ~isDeadLock
            diffs_mesh = diffCurTargetMesh(diffs_mesh, agentPositions, X,Y, MaxValue, r);
            diffs = trapz(y,trapz(x,diffs_mesh,2));
            if diffs_buffer - diffs < 5*1e-3 % condition C1
                [velocities,min_dist,MiniDistPts] = reactionUnderC1( agentPositions,diffs_mesh,X,Y,r,k_linear);
                numberOfZeroControl = numberOfZeroControl+1;
                isDeadLock = true;
            else
                velocities = calculateVelocities(diffs_mesh,k_cov,k_col,k_linear,agentPositions,MaxValue,r,X,Y,x,y);
            end
            velocities  = e_greedyVelocities( agentPositions, velocities, r_communication, epsilon );
            
            
            for i_agent = 1:number_agent
                if(sum(abs(velocities(i_agent,:)))<1e-3)
                    velocities(i_agent,:) = 0.1*(agentPositions(i_agent) - ...
                    (sum(agentPositions(1:i_agent-1,:),1)+sum(agentPositions(i_agent+1:end,:),1))/(number_agent-1)); 
                end
            end
            velocities = CollisionAvoid( velocities, k_col, agentPositions, r*1.5, r*1.2 );
            diffs_buffer = diffs;
            
            velocities = min(max(velocities,-0.2/dt),0.2/dt);
            agentPositions = agentPositions + velocities.*dt;
            agentPositions(:,1) = min(max(agentPositions(:,1),-targetFieldSize(1,1)/2),targetFieldSize(1,1)/2);
            agentPositions(:,2) = min(max(agentPositions(:,2),-targetFieldSize(1,2)/2),targetFieldSize(1,2)/2);
            Results.velocitiesTime{i_time} = velocities;
            Results.diffsTime(1,i_time) = diffs;
            Results.agentPosTime{i_time} = agentPositions;
            if ~isempty(find([timeshares,timeshares*2,timeshares*3,timeshares*4,timeshares*5] == i_time))|| i_time == endTime
                Results.diffMeshTime{find([timeshares,timeshares*2,timeshares*3,timeshares*4,timeshares*5,endTime] == i_time)} = diffs_mesh;
            end
            i_time = i_time+1;
        end
    end
 

end
save('Results.mat','Results')

%% plots
isDrawn = drawFigures(Results,X,Y,number_agent,diffs_mesh);
