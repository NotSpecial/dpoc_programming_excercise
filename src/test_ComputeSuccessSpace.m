function tests = test_ComputeSuccessSpace
%TEST_COMPUTESUCCESSSPACE Test the ComputeSuccessSpace function
%   To run: runtests('test_ComputeSuccessSpace.m')
    tests = functiontests(localfunctions);
end

function test_success(testCase)
testMap = [0 0 1 1];
mansion = [3 1; 4 1];
states = [1 1; 2 1];

expected = [0.25; 0.5];
actual = ComputeSuccessSpace(states, mansion, testMap);
verifyEqual(testCase, actual, expected);
end

function test_obstructed(testCase)
    testMap = [0 1 1];
    mansion = [3 1];
    state = [1 1];
    
    global p_c
    actual = ComputeSuccessSpace(state, mansion, testMap);
    verifyEqual(testCase, actual, p_c);
end

function test_diagonal(testCase)
    testMap = [0 1; 0 0];
    mansion = [2 2];
    state = [0 0];
    
    global p_c
    actual = ComputeSuccessSpace(state, mansion, testMap);
    verifyEqual(testCase, actual, p_c);
end

