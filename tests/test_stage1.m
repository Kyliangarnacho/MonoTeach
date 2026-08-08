function tests = test_stage1
%TEST_STAGE1 Automated regression tests for MonoTeach Stage 1.
%
% Run from MATLAB:
%   results = runtests('tests/test_stage1.m');
% or from stage1/:
%   results = runtests('../tests/test_stage1.m');

    tests = functiontests(localfunctions);
end


function setupOnce(testCase)
    testFile = mfilename('fullpath');
    testsDir = fileparts(testFile);
    repoRoot = fileparts(testsDir);
    stage1Dir = fullfile(repoRoot, 'stage1');

    addpath(stage1Dir);

    testCase.TestData.repoRoot = repoRoot;
    testCase.TestData.stage1Dir = stage1Dir;
    testCase.TestData.robot = build_legacy_robot();
end


function teardownOnce(testCase)
    rmpath(testCase.TestData.stage1Dir);
end


function testRobotStructure(testCase)
    robot = testCase.TestData.robot;

    verifyEqual(testCase, robot.NumBodies, 5);
    verifyEqual(testCase, robot.BodyNames, ...
        {'body1', 'body2', 'body3', 'body4', 'body5'});

    for i = 1:robot.NumBodies
        verifyEqual(testCase, robot.Bodies{i}.Joint.Type, 'revolute');
    end
end


function testHomeConfiguration(testCase)
    robot = testCase.TestData.robot;

    expected = deg2rad([0, 90, 0, 90, 0]);
    actual = homeConfiguration(robot);

    verifyEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end


function testJointLimits(testCase)
    robot = testCase.TestData.robot;

    expectedDeg = [
        -120, 120;
           0, 180;
        -120, 120;
         -30, 210;
        -120, 120
    ];

    for i = 1:robot.NumBodies
        actual = robot.Bodies{i}.Joint.PositionLimits;
        expected = deg2rad(expectedDeg(i, :));
        verifyEqual(testCase, actual, expected, 'AbsTol', 1e-12);
    end
end


function testEveryJointChangesEndEffectorTransform(testCase)
    robot = testCase.TestData.robot;
    qHome = homeConfiguration(robot);
    T0 = getTransform(robot, qHome, 'body5');

    for i = 1:5
        q = qHome;
        q(i) = q(i) + deg2rad(5);
        T = getTransform(robot, q, 'body5');

        verifyGreaterThan(testCase, max(abs(T - T0), [], 'all'), 1e-8);
    end
end


function testFKConsistencyWithPythonReference(testCase)
    robot = testCase.TestData.robot;
    referencePath = fullfile( ...
        testCase.TestData.stage1Dir, ...
        'data', ...
        'python_fk_reference.json');

    verifyTrue(testCase, isfile(referencePath));

    reference = jsondecode(fileread(referencePath));

    for i = 1:numel(reference)
        qRad = deg2rad(reference(i).joint_angles_deg(:)');
        TMatlab = getTransform(robot, qRad, 'body5');
        TPython = reference(i).transform;

        verifyEqual(testCase, TMatlab, TPython, 'AbsTol', 1e-10);
    end
end


function testIKBackSubstitution(testCase)
    robot = testCase.TestData.robot;

    qTarget = deg2rad([30, 60, -30, 120, 45]);
    TTarget = getTransform(robot, qTarget, 'body5');

    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [1, 1, 1, 1, 1, 1];
    initialGuess = homeConfiguration(robot);

    [qIK, solInfo] = ik('body5', TTarget, weights, initialGuess);
    TRecovered = getTransform(robot, qIK, 'body5');

    positionError = norm(TRecovered(1:3, 4) - TTarget(1:3, 4));
    transformError = max(abs(TRecovered - TTarget), [], 'all');

    verifyEqual(testCase, string(solInfo.Status), "success");
    verifyLessThanOrEqual(testCase, positionError, 1e-6);
    verifyLessThanOrEqual(testCase, transformError, 1e-6);
end
