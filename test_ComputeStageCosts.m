function tests = test_ComputeStageCosts
%TEST_COMPUTESTAGECOSTS
%   To run: runtests('test_ComputeStageCosts.m')
    tests = functiontests(localfunctions);
end

function setup(~)
    % Create a small map and compute stage costs for every test
    Pool = - 1;
    Camera = 0.5;
    Mansion = 1;
    Gate = 0;
    Map = [Gate    0 0 Pool;
           Mansion 0 0 Camera];
       
    Mansion = [1 2];
    Cameras = [4 2 0.4];
    Gate = [1 1];
    states = [1 1; 2 1; 3 1; 4 1; 2 2; 3 2];
    
    global G_test;
    G_test = ComputeStageCosts(...
            states, [1:5]', Map, Gate, Mansion, Cameras);
end

function teardown(~)
    clear G_test;
end

function test_pool(testCase)
    global G_test;
    % Chance to end in the pool low, because chance to get detected high!
    % pool state 3, gate state 1
    p_det = 0.8704;
    verifyEqual(testCase, G_test(3, 4), p_det * 7 + (1-p_det) * 1, ...
                'RelTol', 1e-10);
end

function test_picture_safe(testCase)
    global G_test;
    % Taking a picture with no chance of being detected
    verifyEqual(testCase, G_test(2, 5), 1);
end

function test_picture(testCase)
    global G_test;
    % Taking a picture with chance of being detected (state 4)
    verifyEqual(testCase, G_test(6, 5), 0.25 * 1 + 0.75 * (0.4 * 7 + 0.6 * 1));
end


function test_move_safe(testCase)
    global G_test;
    % Moving without Chance of being detected
    verifyEqual(testCase, G_test(5, 3), 1);
end

function test_impossible(testCase)
    global G_test;
    % Impossible Movements have infinite cost
    verifyEqual(testCase, G_test(1, 2), Inf);
    verifyEqual(testCase, G_test(1, 1), Inf);
end

% Add more tests here maybe? Not sure.
