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

% put your code here

n_states = size(stateSpace,1);
n_controls = size(controlSpace, 1);
global detected_additional_time_steps;
global pool_num_time_steps;

% Detection and success probabilities for each state
detectionSpace = ComputeDetectionSpace(stateSpace, cameras, map);
successSpace = ComputeSuccessSpace(stateSpace, mansion, map);

% Initialize G with infinity (infinite costs for impossible moves)
G = Inf * ones(n_states, n_controls);

% Differences to get new state from current state (n w s e)
n_movements = 4;
state_diffs = [0 1; -1 0; 0 -1; 1 0];

% Iterate over states
for i_state = 1:n_states
    state = stateSpace(i_state, :);
    target_states = repmat(state, n_movements, 1) + state_diffs;
    % Not all target states are reachable
    [~, indices] = ismember(target_states, stateSpace, 'rows');
    
    for i_control = 1:n_movements
        i_target = indices(i_control);
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
            
            % chance to be detected (extra cost to move to gate)
            p_det = detectionSpace(i_target);
            
            G(i_state, i_control) = (...
                (cost + detected_additional_time_steps) * p_det + ...
                cost * (1 - p_det));   
        end
    end

    % Taking picture has different costs
    p_success = successSpace(i_state);
    p_detected = detectionSpace(i_state);
    G(i_state, 5) = (...
        p_success * 1 + ...
        (1 - p_success) * (...
            p_detected * (1 + detected_additional_time_steps) + ...
        	(1 - p_detected) * 1 ...
        )...
    );

end
end