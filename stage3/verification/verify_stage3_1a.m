%% MonoTeach Stage 3.1A
% Final integrated verification for the read, task-plane, and retarget bridge.
% This script verifies existing geometry only; it does not run IK or execution.

clear;
clc;

format long g;


%% ------------------------------------------------------------
% 0. Resolve paths
% -------------------------------------------------------------

stage3Dir = fileparts(fileparts(mfilename('fullpath')));
fixturePath = fullfile( ...
    stage3Dir, ...
    'data', ...
    'workspace_trajectory_fixture.json');

addpath(genpath(stage3Dir));

fprintf('============================================\n');
fprintf('MonoTeach Stage 3.1A Final Verification\n');
fprintf('============================================\n');

allPassed = true;


%% ------------------------------------------------------------
% 1. Read the fixed cross-language fixture
% -------------------------------------------------------------

fprintf('\n[1] WorkspaceTrajectory fixture\n');

fixtureExists = isfile(fixturePath);
allPassed = report_result( ...
    allPassed, ...
    'Fixture file exists', ...
    fixtureExists);

if ~fixtureExists
    error( ...
        'MonoTeach:MissingWorkspaceTrajectoryFixture', ...
        'Missing Stage 3.1A fixture: %s', ...
        fixturePath);
end

workspaceTrajectory = load_workspace_trajectory(fixturePath);
workspaceBefore = workspaceTrajectory;

fixtureContractPassed = ...
    strcmp(string(workspaceTrajectory.schema_version), "1.0") && ...
    numel(workspaceTrajectory.samples) == 5 && ...
    sum([workspaceTrajectory.samples.valid]) == 4 && ...
    sum([workspaceTrajectory.samples.inside_workspace]) == 3;

fprintf( ...
    'Fixture samples/valid/inside: %d / %d / %d\n', ...
    numel(workspaceTrajectory.samples), ...
    sum([workspaceTrajectory.samples.valid]), ...
    sum([workspaceTrajectory.samples.inside_workspace]));

allPassed = report_result( ...
    allPassed, ...
    'Fixture loader and contract', ...
    fixtureContractPassed);


%% ------------------------------------------------------------
% 2. Derive a robot-base task trajectory
% -------------------------------------------------------------

fprintf('\n[2] Workspace to robot-base task transform\n');

config = default_task_plane_config();
taskTrajectory = workspace_to_task_trajectory( ...
    workspaceTrajectory, ...
    config);

inputUnchangedPassed = isequaln( ...
    workspaceTrajectory, ...
    workspaceBefore);

allPassed = report_result( ...
    allPassed, ...
    'WorkspaceTrajectory input remains unchanged', ...
    inputUnchangedPassed);


%% ------------------------------------------------------------
% 3. Centre, invalid-gap, and outside semantics
% -------------------------------------------------------------

fprintf('\n[3] TaskTrajectory sample semantics\n');

centerSample = taskTrajectory.samples(2);
centerXYZM = [centerSample.x_m, centerSample.y_m, centerSample.z_m];
anchorErrorM = norm(centerXYZM - config.robot_anchor_m);
centerPassed = centerSample.valid && anchorErrorM <= 1e-12;

fprintf('Centre base XYZ [m]: [%.6f %.6f %.6f]\n', centerXYZM);
fprintf('Anchor error [m]   : %.3e\n', anchorErrorM);

allPassed = report_result( ...
    allPassed, ...
    'Workspace centre maps to robot anchor', ...
    centerPassed);

invalidSample = taskTrajectory.samples(3);
invalidPassed = ...
    ~invalidSample.valid && ...
    ~invalidSample.inside_workspace && ...
    isempty(invalidSample.x_m) && ...
    isempty(invalidSample.y_m) && ...
    isempty(invalidSample.z_m) && ...
    strcmp(string(invalidSample.invalid_reason), "no_hand");

allPassed = report_result( ...
    allPassed, ...
    'Invalid sample preserves empty robot-base XYZ', ...
    invalidPassed);

outsideSample = taskTrajectory.samples(5);
outsideXYZM = [outsideSample.x_m, outsideSample.y_m, outsideSample.z_m];
outsidePassed = ...
    outsideSample.valid && ...
    ~outsideSample.inside_workspace && ...
    all(isfinite(outsideXYZM));

fprintf('Outside base XYZ [m]: [%.6f %.6f %.6f]\n', outsideXYZM);

allPassed = report_result( ...
    allPassed, ...
    'Valid-but-outside sample retains robot-base XYZ', ...
    outsidePassed);


%% ------------------------------------------------------------
% 4. Output coordinate contract and task-plane rotation
% -------------------------------------------------------------

fprintf('\n[4] Coordinate-frame contract\n');

coordinateContractPassed = ...
    strcmp(string(taskTrajectory.coordinate_frame), "robot_base") && ...
    strcmp(string(taskTrajectory.metadata.coordinate_frame), "robot_base") && ...
    strcmp(string(taskTrajectory.metadata.units), "m");

allPassed = report_result( ...
    allPassed, ...
    'TaskTrajectory frame is robot_base and units are m', ...
    coordinateContractPassed);

RBaseTaskplane = config.T_base_taskplane(1:3, 1:3);
rotationOrthogonalityError = max( ...
    abs(RBaseTaskplane' * RBaseTaskplane - eye(3)), ...
    [], ...
    'all');
rotationDeterminant = det(RBaseTaskplane);
rotationPassed = ...
    rotationOrthogonalityError <= 1e-12 && ...
    abs(rotationDeterminant - 1.0) <= 1e-12;

fprintf('Rotation orthogonality error: %.3e\n', rotationOrthogonalityError);
fprintf('Rotation determinant         : %.6f\n', rotationDeterminant);

allPassed = report_result( ...
    allPassed, ...
    'T_base_taskplane rotation is orthonormal', ...
    rotationPassed);


%% ------------------------------------------------------------
% 5. Final result
% -------------------------------------------------------------

fprintf('\n============================================\n');

if allPassed
    fprintf('FINAL RESULT: PASS\n');
else
    fprintf('FINAL RESULT: FAIL\n');
end

fprintf('============================================\n');

if ~allPassed
    error( ...
        'MonoTeach:Stage3_1AVerificationFailed', ...
        'Stage 3.1A verification failed.');
end


%% ------------------------------------------------------------
% Local helper
% -------------------------------------------------------------

function allPassed = report_result(allPassed, label, condition)

    if condition
        result = 'PASS';
    else
        result = 'FAIL';
    end

    fprintf('%s -> %s\n', label, result);
    allPassed = allPassed && condition;
end
