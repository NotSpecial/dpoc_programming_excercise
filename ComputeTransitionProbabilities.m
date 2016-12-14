function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

n_states = size(stateSpace,1);
n_controls = size(controlSpace, 1);
P = zeros(n_states, n_states, n_controls);

% Detection and success probabilities for each state
detectionSpace = ComputeDetectionSpace(stateSpace, cameras, map);
successSpace = ComputeSuccessSpace(stateSpace, mansion, map);

[~, gate_index] = ismember(gate, stateSpace, 'rows');

comparisons = {...
    % North
    @(i,j) stateSpace(i, :) + [0 1] == stateSpace(j, :);
    % West
    @(i,j) stateSpace(i, :) + [-1 0] == stateSpace(j, :);
    % South
    @(i,j) stateSpace(i, :) + [0 -1] == stateSpace(j, :);
    % East
    @(i,j) stateSpace(i, :) + [1 0] == stateSpace(j, :);
    % Picture
    @(i,j) i == j
};

for u = 1:n_controls
    comparison = comparisons{u};   
    for i = 1:n_states
        for j = 1:n_states
            if comparison(i,j)
                if u~=5
                    % Probability for movements
                    p_detected = detectionSpace(j);
                    P(i,j, u) = 1 - p_detected;
                    P(i, gate_index, u) = P(i, gate_index, u) + p_detected;
                else
                    % Probability if taking picture
                    % Assumtion: Taking a successful picture counts as
                    % transition i > i
                    % Note, j == i here
                    p_success = successSpace(j);
                    p_detected = detectionSpace(j);
                    P(i, j, u) = p_success + ...
                                 (1 - p_success) * (1 - p_detected);
                    P(i, gate_index, u) = P(i, gate_index, u) + ...
                                          (1 - p_success) * p_detected;
                end
            end
        end
    end
end

end