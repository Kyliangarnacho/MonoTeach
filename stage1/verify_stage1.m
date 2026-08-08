%% MonoTeach Stage 1
% Final integrated verification
%
% Purpose:
%   1. Build Legacy 5DOF MATLAB robot
%   2. Verify robot structure
%   3. Verify MATLAB FK against Python Stage 0 reference
%   4. Verify MATLAB numerical IK on a known reachable target
%   5. Verify IK result by FK back-substitution
%
% This file does NOT implement a second FK/IK model.
% It only verifies the Stage 1 engineering baseline.

clear;
clc;

format long g;


%% ------------------------------------------------------------
% 0. Resolve Stage 1 directory
% -------------------------------------------------------------

stage1Dir = fileparts(mfilename('fullpath'));

addpath(stage1Dir);

referencePath = fullfile( ...
    stage1Dir, ...
    'data', ...
    'python_fk_reference.json');


fprintf('============================================\n');
fprintf('MonoTeach Stage 1 Final Verification\n');
fprintf('============================================\n');


%% ------------------------------------------------------------
% 1. Build robot
% -------------------------------------------------------------

fprintf('\n[1] Robot structure verification\n');

robot = build_legacy_robot();

allPassed = true;


% 1.1 Body count

bodyCountPassed = ...
    (robot.NumBodies == 5);

fprintf( ...
    'Body count: %d -> %s\n', ...
    robot.NumBodies, ...
    pass_fail(bodyCountPassed));

allPassed = ...
    allPassed && bodyCountPassed;


% 1.2 Body names

expectedBodyNames = ...
    {'body1', 'body2', 'body3', 'body4', 'body5'};

bodyNamesPassed = ...
    isequal(robot.BodyNames, expectedBodyNames);

fprintf( ...
    'Body order -> %s\n', ...
    pass_fail(bodyNamesPassed));

allPassed = ...
    allPassed && bodyNamesPassed;


% 1.3 Joint types

jointTypesPassed = true;

for i = 1:robot.NumBodies

    body = robot.Bodies{i};

    if ~strcmp(body.Joint.Type, 'revolute')
        jointTypesPassed = false;
        break;
    end

end

fprintf( ...
    'All joints revolute -> %s\n', ...
    pass_fail(jointTypesPassed));

allPassed = ...
    allPassed && jointTypesPassed;


% 1.4 Home configuration

expectedHome = ...
    deg2rad([0, 90, 0, 90, 0]);

qHome = ...
    homeConfiguration(robot);

homeError = ...
    max(abs(qHome - expectedHome));

homePassed = ...
    homeError <= 1e-12;

fprintf( ...
    'Home configuration error: %.3e rad -> %s\n', ...
    homeError, ...
    pass_fail(homePassed));

allPassed = ...
    allPassed && homePassed;


%% ------------------------------------------------------------
% 2. Python <-> MATLAB FK verification
% -------------------------------------------------------------

fprintf('\n[2] Python <-> MATLAB FK verification\n');


if ~isfile(referencePath)

    error( ...
        'MonoTeach:MissingFKReference', ...
        ['Missing FK reference file:\n%s\n\n' ...
         'Run from repository root:\n' ...
         'python -m stage1.export_python_fk_reference'], ...
        referencePath);

end


jsonText = ...
    fileread(referencePath);

reference = ...
    jsondecode(jsonText);


fkTolerance = 1e-10;

fkAllPassed = true;

maxObservedFKError = 0;


for i = 1:numel(reference)

    caseName = ...
        reference(i).name;

    qDeg = ...
        reference(i).joint_angles_deg(:)';

    qRad = ...
        deg2rad(qDeg);


    % Python Stage 0 reference

    T_python = ...
        reference(i).transform;

    p_python = ...
        reference(i).position_m(:)';


    % MATLAB Stage 1 FK

    T_matlab = ...
        getTransform( ...
            robot, ...
            qRad, ...
            'body5');

    p_matlab = ...
        T_matlab(1:3, 4)';


    % Errors

    positionError = ...
        max(abs(p_matlab - p_python));

    transformError = ...
        max( ...
            abs(T_matlab - T_python), ...
            [], ...
            'all');

    maxObservedFKError = ...
        max( ...
            maxObservedFKError, ...
            transformError);


    casePassed = ...
        transformError <= fkTolerance;

    fkAllPassed = ...
        fkAllPassed && casePassed;


    fprintf('\n  Case: %s\n', caseName);

    fprintf('  q(deg): ');
    fprintf('%8.3f ', qDeg);
    fprintf('\n');

    fprintf( ...
        '  Position max error : %.3e m\n', ...
        positionError);

    fprintf( ...
        '  Transform max error: %.3e\n', ...
        transformError);

    fprintf( ...
        '  Result: %s\n', ...
        pass_fail(casePassed));

end


fprintf( ...
    '\nFK overall -> %s\n', ...
    pass_fail(fkAllPassed));

allPassed = ...
    allPassed && fkAllPassed;


%% ------------------------------------------------------------
% 3. MATLAB numerical IK verification
% -------------------------------------------------------------

fprintf('\n[3] MATLAB numerical IK verification\n');


% Generate a target from a known legal robot configuration.
% Therefore TTarget is guaranteed to be reachable.

qTargetReference = ...
    deg2rad([30, 60, -30, 120, 45]);

TTarget = ...
    getTransform( ...
        robot, ...
        qTargetReference, ...
        'body5');


% Build numerical IK solver

ik = ...
    inverseKinematics( ...
        'RigidBodyTree', robot);


% Orientation XYZ + Translation XYZ weights

weights = ...
    [1, 1, 1, 1, 1, 1];


% Numerical solver starts searching near Home

initialGuess = ...
    homeConfiguration(robot);


[qIK, solInfo] = ...
    ik( ...
        'body5', ...
        TTarget, ...
        weights, ...
        initialGuess);


% FK back-substitution

TRecovered = ...
    getTransform( ...
        robot, ...
        qIK, ...
        'body5');


positionErrorIK = ...
    norm( ...
        TRecovered(1:3, 4) - ...
        TTarget(1:3, 4));


transformErrorIK = ...
    max( ...
        abs(TRecovered - TTarget), ...
        [], ...
        'all');


ikPositionTolerance = ...
    1e-6;

ikTransformTolerance = ...
    1e-6;


ikStatusPassed = ...
    strcmpi( ...
        string(solInfo.Status), ...
        "success");

ikPositionPassed = ...
    positionErrorIK <= ikPositionTolerance;

ikTransformPassed = ...
    transformErrorIK <= ikTransformTolerance;


ikPassed = ...
    ikStatusPassed && ...
    ikPositionPassed && ...
    ikTransformPassed;


fprintf( ...
    'IK status          : %s\n', ...
    string(solInfo.Status));

fprintf( ...
    'IK iterations      : %d\n', ...
    solInfo.Iterations);

fprintf( ...
    'Position error     : %.3e m\n', ...
    positionErrorIK);

fprintf( ...
    'Transform error    : %.3e\n', ...
    transformErrorIK);

fprintf('IK solution (deg):\n');
disp(rad2deg(qIK));

fprintf( ...
    'IK verification -> %s\n', ...
    pass_fail(ikPassed));


allPassed = ...
    allPassed && ikPassed;


%% ------------------------------------------------------------
% 4. Final result
% -------------------------------------------------------------

fprintf('\n============================================\n');

fprintf( ...
    'Maximum FK transform error: %.3e\n', ...
    maxObservedFKError);

fprintf( ...
    'IK position error          : %.3e m\n', ...
    positionErrorIK);

fprintf( ...
    'IK transform error         : %.3e\n', ...
    transformErrorIK);


if allPassed

    fprintf('\nFINAL RESULT: PASS\n');

else

    fprintf('\nFINAL RESULT: FAIL\n');

end

fprintf('============================================\n');


if ~allPassed

    error( ...
        'MonoTeach:Stage1VerificationFailed', ...
        'Stage 1 verification failed.');

end


%% ------------------------------------------------------------
% Local helper
% -------------------------------------------------------------

function result = pass_fail(condition)

    if condition
        result = 'PASS';
    else
        result = 'FAIL';
    end

end