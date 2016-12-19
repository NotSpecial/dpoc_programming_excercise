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