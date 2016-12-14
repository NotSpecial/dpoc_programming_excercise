function detectionSpace = ComputeDetectionSpace(stateSpace, cameras, map)
    n_states = size(stateSpace,1);
    detectionSpace = zeros(n_states, 1);
    for i = 1:n_states
        detectionSpace(i) = detection_probability(stateSpace(i,:),...
                                                  cameras, map);
    end
end

function p = detection_probability (state, cameras, map)
    % Probability to be detected by all cameras at a certain state
    distances = FreeDistances(state, cameras, map);
    
    p = cameras(:, 3) ./ distances;
    
    % If in pond camera can check 4 times
    global pool_num_time_steps;
    % Careful with map indexing! Coordinates need to be swapped.
    if map(state(2), state(1)) < 0
        p = 1 - (1 - p) .^ pool_num_time_steps;
    end
    
    % Probability to be seen by any camera:
    % 1 - probabilty to be not seen by any camera
    p = 1 - prod( (1 - p) );
    
end