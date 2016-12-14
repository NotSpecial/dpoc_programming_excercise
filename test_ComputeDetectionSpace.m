function tests = test_ComputeDetectionSpace
%TEST_COMPUTEDETECTIONSPACE Test the ComputeDetectionSpace function
%   To run: runtests('test_ComputeDetectionSpace.m')
    tests = functiontests(localfunctions);
end

% We have already tested that distances work for all directions and
% obstacles. We will ignore those things in the tests now

function test_weight(testCase)
    testMap = zeros(1, 3);
    camera = [3, 1, 0.5];
    testMap(1, 3) = 0.1234;
    
    states = [1 1; 2 1];
    
    % Weight/Distance
    expected = [0.25; 0.5];
    actual = ComputeDetectionSpace(states, camera, testMap);
    verifyEqual(testCase, actual, expected);
end

function test_obstructed(testCase)
    testMap = zeros(1, 3);
    camera = [3, 1, 0.5];
    testMap(1, 3) = 0.1234;
    testMap(1, 2) = 1;  % Obstacle
    
    states = [1 1];
    expected = 0;  % Can't be seen, no chance of detection
    actual = ComputeDetectionSpace(states, camera, testMap);
    verifyEqual(testCase, actual, expected);
end

function test_diagonal(testCase)
    testMap = [0 1;
               0 0];
    camera = [2 2 1];
    state = [1 1];
    
    expected = 0;
    actual = ComputeDetectionSpace(state, camera, testMap);
    verifyEqual(testCase, actual, expected);
end

function test_pool(testCase)
    testMap = [-2 1];  % pool, camera
    camera = [2 1 0.4];
    state = [1 1];
    
    % Camera has 4 chances to detect, probability 1 - (1-p)^4 where p is
    % gamma/d
    expected = 0.8704;
    actual = ComputeDetectionSpace(state, camera, testMap);
    verifyEqual(testCase, actual, expected, 'RelTol', 1e-10);
end

function test_multiple_cams(testCase)
    % Probability to get seen by multiply cameras
    % This is not just the sum (this could exceed 100% probability!)
    testMap = [1 0 1];
    cameras = [1 1 0.5; 3 1 0.25];
    state = [2 1];
    
    % Expected = 1 - chance to get seen by no camera
    expected = 0.625;
    actual = ComputeDetectionSpace(state, cameras, testMap);
    verifyEqual(testCase, actual, expected);
end
