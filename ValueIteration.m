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
global timer;
tic;
% Initialize J_opt and u_opt_ind
% Use approximate map size as initialization (WIP)
K = size(P, 1);
J_opt = sqrt(K) * ones(K, 1);
u_opt_ind = zeros(K, 1);
tolerance = 1e-5;

% Debugging
global J_debug
J_debug = J_opt;

while 1
    J_old = J_opt;   
    
    for i = 1:K
        [J_opt(i), u_opt_ind(i)] = ...
            min(G(i,:) + J_opt' * squeeze(P(i, :, :)));
    end
    
    % debugging
    J_debug = [J_debug J_opt];
    
    if max(abs(J_opt - J_old)) <= tolerance
        break;
    end
end

timer = toc;
end