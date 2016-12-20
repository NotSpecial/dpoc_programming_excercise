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