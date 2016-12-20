function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.

global debug
start = tic;

n_states = size(stateSpace,1);
global detected_additional_time_steps;
global pool_num_time_steps;

% Detection and success probabilities for each state
detectionSpace = ComputeDetectionSpace(stateSpace, cameras, map);
successSpace = ComputeSuccessSpace(stateSpace, mansion, map);

% Initialize G with infinity (infinite costs for impossible moves)
G = Inf * ones(n_states, 5);

% Differences to get new state from current state (n w s e)
n_movements = 4;
state_diffs = [0 1; -1 0; 0 -1; 1 0];

% Iterate over states
for i_state = 1:n_states
    % Control Input: Move
    % Get target state indices for each movement
    % Not all targets are valid states (index will be 0)
    % Add position difference to state and look if result is in stateSpace
    state = stateSpace(i_state, :);
    target_states = repmat(state, n_movements, 1) + state_diffs;
    [~, indices] = ismember(target_states, stateSpace, 'rows');
    
    for u = 1:n_movements
        i_target = indices(u);
        if  i_target ~= 0
            % Check if target is a pool, this will increase costs
            % Careful: While indexing the map, we need to reverse the state
            % Thats why we use [2 1] to swap the elements
            if map(stateSpace(i_target, [2 1])) >= 0
                % no pool
                cost = 1;
            else
                cost = pool_num_time_steps;
            end
            
            % chance to be detected (extra cost if detected)
            p_det = detectionSpace(i_target);
            
            G(i_state, u) = (...
                p_det * (cost + detected_additional_time_steps)  + ...
                (1 - p_det) * cost);   
        end
    end

    % Taking picture has different costs
    p_success_pic = successSpace(i_state);
    p_detected_pic = detectionSpace(i_state);
    G(i_state, 5) = (...
        p_success_pic * 1 + ...
        (1 - p_success_pic) * (...
            p_detected_pic * (1 + detected_additional_time_steps) + ...
        	(1 - p_detected_pic) * 1 ...
        )...
    );

debug.time_stagecosts = toc(start);

end
end

function successSpace = ComputeSuccessSpace(stateSpace, mansion, map)
    n_states = size(stateSpace,1);
    successSpace = zeros(n_states, 1);
    for i = 1:n_states
        successSpace(i) = success_probability(stateSpace(i,:),...
                                              mansion, map);
    end
end

function p = success_probability(state, mansion, map)
    % distances to mansion squares
    distances = FreeDistances(state, mansion, map);
    
    % quality of paparazzo camera, minumum probability of success
    global gamma_p
    global p_c
    
    p_mansion = gamma_p ./ distances;
    
    % Take probabilities of all mansion squares and minimum probability and
    % take maximum
    p = max([p_c; p_mansion]);
    
end

function detectionSpace = ComputeDetectionSpace(stateSpace, cameras, map)
    % Probability to be detected at each state
    n_states = size(stateSpace,1);
    detectionSpace = zeros(n_states, 1);
    for i = 1:n_states
        detectionSpace(i) = detection_probability(stateSpace(i,:),...
                                                  cameras, map);
    end
end

function p = detection_probability (state, cameras, map)
    % Distances to all cameras at given state
    distances = FreeDistances(state, cameras, map);
    
    % Array of probabilities to NOT be detected by each camera
    p_not = 1 - (cameras(:, 3) ./ distances);
    
    % If in pond camera can check 4 times
    % Probability to not be detected is simply to the power of 4
    % Careful with map indexing! Coordinates need to be swapped.
    global pool_num_time_steps;
    if map(state(2), state(1)) < 0
        p_not = p_not .^ pool_num_time_steps;
    end
    
    % Probability to be seen by any camera or several of them:
    % 1 - probabilty to be not seen by any camera
    p = 1 - prod( p_not );
end

function [ distances ] = FreeDistances( currState, targetCoordinates, map )
%FREEDISTANCES Distances from target states if view is free.
% 	Compute the distance for every state in targetStates if no tree or bush
% 	is obstructing the field of view. If the view is obstructed, the
% 	distance will be infinite.

n_targets = size(targetCoordinates, 1);
distances = Inf * ones(n_targets,1);

for i = 1:n_targets
    target = targetCoordinates(i,1:2);
   if freeView(currState, target, map)
        distances(i) = max(abs(currState - target));
   end
end

end

function [ free ] = freeView( currState, target, map )
%FREEVIEW returns true, iff x or y coordinates of P1(x,y) and P2(x,y) align
% and if there is no obstacle in line of sight between them
if any( currState - target == 0)
    % Aligned in n direction:
    if currState(1)==target(1)
        range_n = currState(1);
        if currState(2)>target(2)
            range_m = target(2)+1 : currState(2)-1;
        else
            range_m = currState(2)+1 : target(2)-1;
        end 
    end

    % Aligned in m direction:
    if currState(2)==target(2); %ycoord of points
        range_m = currState(2);
        if currState(1)>target(1)
            range_n = target(1)+1 : currState(1)-1;
        else
            range_n = currState(1)+1 : target(1)-1;
        end
    end

    free = ~any( map(range_m, range_n) > 0 );
else
    free = 0;
end
end
