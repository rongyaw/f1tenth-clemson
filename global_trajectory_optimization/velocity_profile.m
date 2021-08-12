function [Vel, Time] = velocity_profile(x, y, yaw, mu, v_max, v_min, num_state)
% Calculate the change of position and orientation
dx = x(2:end) - x(1:end-1);
dy = y(2:end) - y(1:end-1);
dyaw = yaw(2:end) - yaw(1:end-1);

% Yaw correction of delta_yaw
for i = 1:1:length(dyaw)
    if dyaw(i) > pi
        dyaw(i) = dyaw(i) - 2*pi;
    else
        if dyaw(i) < -pi
            dyaw(i) = dyaw(i) + 2*pi;
        else
            dyaw(i) = dyaw(i);
        end
    end
end

dl = (dx.^2 + dy.^2).^0.5;
R = (dl/2)./sin(dyaw/2);
ds = smoothdata(dyaw.*R);

% Initialize cost and index matrix
V = linspace(v_min, v_max, num_state);
[V_current, V_previous] = meshgrid(V);
V_avg = (V_current + V_previous)/2;
dV = (V_current - V_previous);
J = zeros(num_state, length(ds)+1);
J_index = zeros(num_state, length(ds));

progress = 0.0;

for i = length(ds):-1:1
    % Calculate Tire Force
    dt = ds(i)./V_avg;
    a_long = dV./dt;
    a_long(a_long > 7.51) = inf;
    a_long(a_long < -8.26) = inf;
    
    w = dyaw(i)./dt;
    %w(w > 6.2) = inf;
    a_tire = (a_long.^2 + (w.*V_avg).^2).^0.5;
    
    % Apply tire force constraint
    a_tire(a_tire > mu*9.8) = inf;
    a_tire(a_tire <= mu*9.8) = 1;
    
    % Calculate time cost
    cost = dt.*a_tire;
    J_prev = repmat(J(:, i+1), 1, num_state);
    J_current = J_prev + cost;
    
    % Find minimum cost value and index
    [J_min, J_min_id] = min(J_current);
    J(:, i) = J_min;
    J_index(:, i) = J_min_id;
    
    % Output the DP progress
    p = (length(ds) - i)/length(ds)*100;
    if p-progress > 10
        progress = progress + 10;
        clc
        disp(['The DP is ',num2str(progress),'% down.'])
    end
end
clc 
disp('The DP optimization is complete.')

% Record the optimal velocity profile
Vel = [];
Time = [];
[min_cost, min_cost_id] = min(J(:,1));
Time = min_cost;
disp(['The minimum lap time is ', num2str(Time), ' seconds'])
current_id = min_cost_id;
Vel = [Vel, V(current_id)];
previous_id = J_index(min_cost_id, 1);
for i = 2:1:length(x)-1
    current_id = previous_id;
    Vel = [Vel, V(current_id)];
    previous_id = J_index(current_id, i);
end
% Smooth out the velocity profile
Vel = smooth([Vel, Vel(end)], 0.01, 'rloess');
disp(['Maximum speed is ', num2str(max(Vel)), ' m/s and minimum speed is ', num2str(min(Vel)), ' m/s.'])
%Vel = [Vel, Vel(end)];
end

