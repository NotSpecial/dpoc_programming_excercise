function tests = test_ComputeTransitionProbabilities
%TEST_COMPUTETRANSITIONPROBABILITIES
%   To run: runtests('test_ComputeTransitionProbabilties.m')
    tests = functiontests(localfunctions);
end

function test_all(testCase)
    % Create a small map to test different effects
    Pool = - 1;
    Camera = 0.5;
    Mansion = 1;
    Gate = 0;
    Map = [Gate    0 Pool;
           Mansion 0 Camera];
       
    Mansion = [1 2];
    Cameras = [3 2 0.5];
    Gate = [1 1];
    states = [1 1; 2 1; 3 1; 2 2];
    
    P = ComputeTransitionProbabilities(...
            states, [1:5]', Map, Gate, Mansion, Cameras);
    
    % Going into pool
    % Chance to end in the pool low, because chance to get detected high!
    % pool state 3, gate state 1
    p_detected = 0.9375;
    verifyEqual(testCase, P(2, 3, 4), 1 - p_detected);
    verifyEqual(testCase, P(2, 1, 4), p_detected);
    
    % Taking a picture with no chance of being detected
    verifyEqual(testCase, P(2, 2, 5), 1);
    
    % Taking a picture with chance of being detected (state 4)
    p_detected = 0.25;  % Can only be detected if no success
    verifyEqual(testCase, P(4, 4, 5), 1 - p_detected);
    verifyEqual(testCase, P(4, 1, 5), p_detected);
    
    % Moving without Chance of being detected
    verifyEqual(testCase, P(4, 2, 3), 1);
    
    % Impossible Movements have chance 0
    verifyEqual(testCase, P(1, 3, 4), 0);
    verifyEqual(testCase, P(1, 2, 1), 0);
    
    % Add more tests here maybe? Not sure.
end