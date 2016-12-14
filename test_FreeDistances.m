function tests = test_FreeDistances
%TEST_FREEDISTANCES Test the FreeDistances function
%   To run: runtests('test_FreeDistances.m')
    tests = functiontests(localfunctions);
end

function test_distances(testCase)
% Test that distances are computed correctly for all directions
    % test map has one block mansion in the middle, nothing else
    mansion = [3 3];

    testMap = zeros(5, 5);
    testMap(mansion) = 1;

    % 4 states in all directions with distance 1
    d1 = [2 3; 4 3; 3 2; 3 4];
    for i=1:length(d1)
        expected = 1;
        actual = FreeDistances(d1(i, :), mansion, testMap);
        verifyEqual(testCase, actual, expected);
    end

    % 4 more states with distance 2
    d2 = [1 3; 5 3; 3 1; 3 5];
    for i=1:length(d2)
        expected = 2; 
        actual = FreeDistances(d2(i, :), mansion, testMap);
        verifyEqual(testCase, actual, expected);
    end
end

function test_obstacles(testCase)
% Test that obstacles lead to distance infinity
    % Like test before, but with obstacles around the mansion
    mansion = [3 3];
    testMap = zeros(5, 5);
    testMap(2:4, 2:4) = 1;  %ones all around, middle can't be seen
    
    % 4 states behind the obstacles
    positions = [1 3; 5 3; 3 1; 3 5];
    for i=1:length(positions)
        expected = Inf; 
        actual = FreeDistances(positions(i, :), mansion, testMap);
        verifyEqual(testCase, actual, expected);
    end
end

function test_no_diagonal(testCase)
% Test that distances on the diagonal are alwazs infinity
    % Setup: Cameras at all corners, state in the middle.
    testMap = ones(3,3);
    testMap(2, :) = 0;
    testMap(:, 2) = 0;  % Now only the corners are still 1

    cameras = [1 1; 1 3; 3 1; 3 3];
    n_cams = length(cameras);
    position = [2 2];

    expected = Inf * ones(n_cams, 1);
    actual = FreeDistances(position, cameras, testMap);
    verifyEqual(testCase, actual, expected);
end

function test_multiple(testCase)
% Test that the function is correctly applied to several cameras etc.
    % Setup: 4 cameras in different distances and directions
    testMap = zeros(9, 9);
    cameras = [4 5; 5 3; 8 5; 5 9; 9 9];
    n_cams = length(cameras);
    for i=1:n_cams
        testMap(cameras(i, [2 1])) = 0.5;
    end
    
    position = [5 5];  % Middle
    
    expected = [1; 2; 3; 4; Inf];
    actual = FreeDistances(position, cameras, testMap);
    verifyEqual(testCase, actual, expected);
end

