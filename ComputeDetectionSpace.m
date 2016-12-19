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