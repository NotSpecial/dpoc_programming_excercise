function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% Debugging
global debug;
start = tic;

% Initialize J_opt and u_opt_ind
% Initializing J_opt with 10 seems to work really well in most situations
K = size(P, 1);
L = size(P, 3);
J_opt = 10 * ones(K, 1);
u_opt_ind = zeros(K, 1);
tolerance = 1e-5;

% Debugging
J_debug = J_opt;

% Array for temporary equations to find minimum
temp = zeros(K, L);

while 1
    J_old = J_opt;   

    % Iterating over control inputs instead of states scales much better
    for u=1:L
        temp(:, u) = G(:, u) + squeeze(P(:, :, u)) * J_opt;
    end
    % Find optimal costs and control
    [J_opt, u_opt_ind] = min(temp, [], 2);
    
    % debugging
    J_debug = [J_debug J_opt];
    
    if max(abs(J_opt - J_old)) <= tolerance
        break;
    end
end

debug.time_valueit = toc(start);
debug.J_val = J_debug;
end