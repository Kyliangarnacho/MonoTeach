function tests = test_stage3_1
%TEST_STAGE3_1 Cross-language bridge and pre-IK tests for MonoTeach Stage 3.1.
%
% Run from MATLAB at repository root:
%   results = runtests('tests/test_stage3_1.m');

    tests = functiontests(localfunctions);
end


function setupOnce(testCase)

    testFile = mfilename('fullpath');
    testsDir = fileparts(testFile);
    repoRoot = fileparts(testsDir);
    stage3Dir = fullfile(repoRoot, 'stage3');
    stage1Dir = fullfile(repoRoot, 'stage1');
    fixturePath = fullfile( ...
        stage3Dir, ...
        'data', ...
        'workspace_trajectory_fixture.json');

    stage3Path = genpath(stage3Dir);
    addpath(stage3Path);
    addpath(stage1Dir);

    testCase.TestData.stage3Dir = stage3Dir;
    testCase.TestData.stage3Path = stage3Path;
    testCase.TestData.stage1Dir = stage1Dir;
    testCase.TestData.fixturePath = fixturePath;
    testCase.TestData.trajectory = load_workspace_trajectory(fixturePath);
    testCase.TestData.taskPlaneConfig = default_task_plane_config();
    testCase.TestData.taskTrajectory = workspace_to_task_trajectory( ...
        testCase.TestData.trajectory, ...
        testCase.TestData.taskPlaneConfig);
end


function teardownOnce(testCase)

    rmpath(testCase.TestData.stage3Path);
    rmpath(testCase.TestData.stage1Dir);
end


function testFixtureLoadsWithSupportedSchema(testCase)

    trajectory = testCase.TestData.trajectory;

    verifyEqual(testCase, string(trajectory.schema_version), "1.0");
end


function testLegacy5RobotContextMatchesFrozenModel(testCase)

    context = load_robot_context("legacy5");
    legacyModel = build_legacy_robot();

    verifyEqual(testCase, context.id, 'legacy5');
    verifyEqual(testCase, context.backend, 'matlab_rigidbodytree');
    verifyEqual(testCase, context.dof, numel(homeConfiguration(legacyModel)));
    verifyEqual(testCase, context.model.NumBodies, legacyModel.NumBodies);
    verifyEqual(testCase, context.home_q, homeConfiguration(legacyModel), ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, context.end_effector, 'body5');
    expectedLimits = zeros(context.dof, 2);
    expectedNames = cell(1, context.dof);
    for index = 1:context.dof
        expectedLimits(index, :) = legacyModel.Bodies{index}.Joint.PositionLimits;
        expectedNames{index} = legacyModel.Bodies{index}.Joint.Name;
    end
    verifyEqual(testCase, context.joint_limits, expectedLimits, 'AbsTol', 1e-12);
    verifyEqual(testCase, context.joint_names, expectedNames);
    config = default_timed_joint_trajectory_config(context);
    verifyEqual(testCase, config.max_velocity_rad_s, context.velocity_limits, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, config.max_acceleration_rad_s2, ...
        context.acceleration_limits, 'AbsTol', 1e-12);
end


function testCanonicalTaskRetargetAdapterPreservesLegacyMapping(testCase)

    workspace = testCase.TestData.trajectory;
    workspaceBefore = workspace;
    context = load_robot_context("legacy5");
    task = workspace_to_canonical_task(workspace);
    taskBefore = task;
    robotTask = retarget_task_to_robot(task, context, ...
        testCase.TestData.taskPlaneConfig);
    legacyTask = workspace_to_task_trajectory(workspace, ...
        testCase.TestData.taskPlaneConfig);

    verifyEqual(testCase, workspace, workspaceBefore);
    verifyEqual(testCase, task, taskBefore);
    verifyEqual(testCase, task.artifact_type, 'CanonicalTaskTrajectory');
    verifyEqual(testCase, string(task.coordinate_frame), "workspace_2d");
    verifyEqual(testCase, robotTask.artifact_type, 'RobotTargetTrajectory');
    verifyEqual(testCase, robotTask.robot_id, context.id);
    verifyEqual(testCase, robotTask.coordinate_frame, legacyTask.coordinate_frame);
    verifyEqual(testCase, robotTask.metadata, legacyTask.metadata);
    verifyEqual(testCase, robotTask.samples, legacyTask.samples);
    adaptedSegments = build_preik_segments(robotTask);
    legacySegments = build_preik_segments(legacyTask);
    verifyEqual(testCase, adaptedSegments.summary, legacySegments.summary);
    verifyEqual(testCase, {adaptedSegments.segments.source_indices}, ...
        {legacySegments.segments.source_indices});
end


function testFixtureMetadata(testCase)

    metadata = testCase.TestData.trajectory.metadata;

    verifyEqual(testCase, ...
        string(metadata.source_trajectory_id), ...
        "stage3-1a-fixture");

    verifyEqual(testCase, ...
        string(metadata.workspace_calibration_id), ...
        "workspace_2d_fixture_190x290");

    verifyEqual(testCase, string(metadata.coordinate_frame), "workspace_2d");
    verifyEqual(testCase, metadata.width_mm, 190.0, 'AbsTol', 1e-12);
    verifyEqual(testCase, metadata.height_mm, 290.0, 'AbsTol', 1e-12);
end


function testFixtureSampleCounts(testCase)

    samples = testCase.TestData.trajectory.samples;

    verifyEqual(testCase, numel(samples), 5);
    verifyEqual(testCase, sum([samples.valid]), 4);
    verifyEqual(testCase, sum([samples.inside_workspace]), 3);
end


function testInvalidSamplePreservesGapSemantics(testCase)

    invalidSample = testCase.TestData.trajectory.samples(3);

    verifyFalse(testCase, invalidSample.valid);
    verifyEmpty(testCase, invalidSample.x_mm);
    verifyEmpty(testCase, invalidSample.y_mm);
    verifyFalse(testCase, invalidSample.inside_workspace);
    verifyEqual(testCase, string(invalidSample.invalid_reason), "no_hand");
end


function testOutsideSampleRemainsValid(testCase)

    outsideSample = testCase.TestData.trajectory.samples(5);

    verifyTrue(testCase, outsideSample.valid);
    verifyFalse(testCase, outsideSample.inside_workspace);
    verifyEqual(testCase, outsideSample.x_mm, 205.0, 'AbsTol', 1e-12);
    verifyEqual(testCase, outsideSample.y_mm, 100.0, 'AbsTol', 1e-12);
    verifyEmpty(testCase, outsideSample.invalid_reason);
end


function testWorkspaceCenterMapsToRobotAnchor(testCase)

    taskTrajectory = workspace_to_task_trajectory( ...
        testCase.TestData.trajectory, ...
        testCase.TestData.taskPlaneConfig);

    centerSample = taskTrajectory.samples(2);
    expectedAnchorM = testCase.TestData.taskPlaneConfig.robot_anchor_m;

    verifyTrue(testCase, centerSample.valid);
    verifyEqual( ...
        testCase, ...
        [centerSample.x_m, centerSample.y_m, centerSample.z_m], ...
        expectedAnchorM, ...
        'AbsTol', 1e-12);
end


function testTaskTrajectoryPreservesInvalidGap(testCase)

    taskTrajectory = workspace_to_task_trajectory( ...
        testCase.TestData.trajectory, ...
        testCase.TestData.taskPlaneConfig);

    invalidSample = taskTrajectory.samples(3);

    verifyFalse(testCase, invalidSample.valid);
    verifyFalse(testCase, invalidSample.inside_workspace);
    verifyEqual(testCase, string(invalidSample.invalid_reason), "no_hand");
    verifyEmpty(testCase, invalidSample.x_m);
    verifyEmpty(testCase, invalidSample.y_m);
    verifyEmpty(testCase, invalidSample.z_m);
end


function testOutsideSampleRemainsValidAfterTaskTransform(testCase)

    taskTrajectory = workspace_to_task_trajectory( ...
        testCase.TestData.trajectory, ...
        testCase.TestData.taskPlaneConfig);

    outsideSample = taskTrajectory.samples(5);

    verifyTrue(testCase, outsideSample.valid);
    verifyFalse(testCase, outsideSample.inside_workspace);
    verifyEqual( ...
        testCase, ...
        [outsideSample.x_m, outsideSample.y_m, outsideSample.z_m], ...
        [0.155, 0.040, 0.2775], ...
        'AbsTol', 1e-12);
end


function testTaskTransformDoesNotModifyWorkspaceTrajectory(testCase)

    workspaceTrajectory = testCase.TestData.trajectory;
    workspaceBefore = workspaceTrajectory;

    workspace_to_task_trajectory( ...
        workspaceTrajectory, ...
        testCase.TestData.taskPlaneConfig);

    verifyEqual(testCase, workspaceTrajectory, workspaceBefore);
end


function testTaskTrajectoryRecordsRobotBaseMetreContract(testCase)

    taskTrajectory = workspace_to_task_trajectory( ...
        testCase.TestData.trajectory, ...
        testCase.TestData.taskPlaneConfig);

    verifyEqual(testCase, string(taskTrajectory.coordinate_frame), "robot_base");
    verifyEqual( ...
        testCase, ...
        string(taskTrajectory.metadata.source_coordinate_frame), ...
        "workspace_2d");
    verifyEqual( ...
        testCase, ...
        string(taskTrajectory.metadata.coordinate_frame), ...
        "robot_base");
    verifyEqual(testCase, string(taskTrajectory.metadata.units), "m");
    verifyEqual( ...
        testCase, ...
        taskTrajectory.metadata.scale_m_per_mm, ...
        testCase.TestData.taskPlaneConfig.scale, ...
        'AbsTol', 1e-12);
end


function testPreIkEligibilityBuildsOnlyContiguousInsideRuns(testCase)

    segmentSet = build_preik_segments(testCase.TestData.taskTrajectory);

    verifyEqual( ...
        testCase, ...
        segmentSet.eligible_mask, ...
        logical([true, true, false, true, false]));

    verifyEqual(testCase, segmentSet.summary.eligible_sample_count, 3);
    verifyEqual(testCase, segmentSet.summary.barrier_sample_count, 2);
    verifyEqual(testCase, segmentSet.summary.segment_count, 2);
    verifyEqual(testCase, segmentSet.summary.single_point_segment_count, 1);

    verifyEqual(testCase, numel(segmentSet.segments), 2);
    verifyEqual(testCase, segmentSet.segments(1).start_index, 1);
    verifyEqual(testCase, segmentSet.segments(1).end_index, 2);
    verifyEqual(testCase, segmentSet.segments(1).source_indices, [1, 2]);
    verifyEqual(testCase, ...
        [segmentSet.segments(1).samples.t_ms], ...
        [0.0, 100.0]);
    verifyEqual(testCase, segmentSet.segments(1).start_t_ms, 0.0);
    verifyEqual(testCase, segmentSet.segments(1).end_t_ms, 100.0);

    verifyEqual(testCase, segmentSet.segments(2).start_index, 4);
    verifyEqual(testCase, segmentSet.segments(2).end_index, 4);
    verifyEqual(testCase, segmentSet.segments(2).source_indices, 4);
    verifyEqual(testCase, numel(segmentSet.segments(2).samples), 1);
    verifyEqual(testCase, segmentSet.segments(2).start_t_ms, 300.0);
    verifyEqual(testCase, segmentSet.segments(2).end_t_ms, 300.0);
end


function testPreIkBarriersPreserveSourceEvidence(testCase)

    taskTrajectory = testCase.TestData.taskTrajectory;
    segmentSet = build_preik_segments(taskTrajectory);
    includedIndices = [segmentSet.segments.source_indices];

    verifyFalse(testCase, ismember(3, includedIndices));
    verifyFalse(testCase, ismember(5, includedIndices));

    invalidSample = taskTrajectory.samples(3);
    verifyFalse(testCase, invalidSample.valid);
    verifyEmpty(testCase, invalidSample.x_m);
    verifyEmpty(testCase, invalidSample.y_m);
    verifyEmpty(testCase, invalidSample.z_m);

    outsideSample = taskTrajectory.samples(5);
    verifyTrue(testCase, outsideSample.valid);
    verifyFalse(testCase, outsideSample.inside_workspace);
    verifyEqual( ...
        testCase, ...
        [outsideSample.x_m, outsideSample.y_m, outsideSample.z_m], ...
        [0.155, 0.040, 0.2775], ...
        'AbsTol', 1e-12);
end


function testPreIkSegmentationDoesNotModifyTaskTrajectory(testCase)

    taskTrajectory = testCase.TestData.taskTrajectory;
    taskTrajectoryBefore = taskTrajectory;

    segmentSet = build_preik_segments(taskTrajectory);

    verifyEqual(testCase, taskTrajectory, taskTrajectoryBefore);
    verifyEqual(testCase, string(segmentSet.coordinate_frame), "robot_base");
    verifyEqual(testCase, string(segmentSet.units), "m");
end


function testLegacyTaskSemanticsDefaultToOneDownStroke(testCase)

    legacyWorkspace = testCase.TestData.trajectory;
    legacyWorkspace.samples = legacyWorkspace.samples(1:2);
    legacyWorkspaceBefore = legacyWorkspace;

    canonicalTask = workspace_to_canonical_task(legacyWorkspace);
    semanticStrokeSet = derive_semantic_strokes(canonicalTask);

    verifyEqual(testCase, legacyWorkspace, legacyWorkspaceBefore);
    verifyEqual(testCase, ...
        string(canonicalTask.metadata.task_semantics.semantic_source), ...
        "legacy_single_stroke_default");
    verifyTrue(testCase, canonicalTask.metadata.task_semantics.legacy_default_applied);
    verifyEqual(testCase, string({canonicalTask.samples.pen_state}), ...
        ["DOWN", "DOWN"]);
    verifyEqual(testCase, [canonicalTask.samples.stroke_id], [1, 1]);
    verifyEqual(testCase, semanticStrokeSet.summary.semantic_stroke_count, 1);
    verifyEqual(testCase, semanticStrokeSet.strokes(1).stroke_id, 1);
    verifyEqual(testCase, semanticStrokeSet.strokes(1).source_indices, [1, 2]);
end


function testExplicitUpBoundarySeparatesSemanticStrokes(testCase)

    samples = repmat(struct('pen_state', '', 'stroke_id', NaN), 1, 5);
    penStates = {'DOWN', 'DOWN', 'UP', 'DOWN', 'DOWN'};
    strokeIds = [1, 1, NaN, 2, 2];
    for i = 1:numel(samples)
        samples(i).pen_state = penStates{i};
        samples(i).stroke_id = strokeIds(i);
    end
    task = struct( ...
        'artifact_type', 'CanonicalTaskTrajectory', ...
        'samples', samples);
    taskBefore = task;

    % Both DOWN runs may be IK-successful, but their UP boundary remains a
    % task semantic boundary rather than an execution-segment decision.
    successfulExecutionSourceIndices = [1, 2, 4, 5];
    verifyEqual(testCase, numel(successfulExecutionSourceIndices), 4);
    semanticStrokeSet = derive_semantic_strokes(task);

    verifyEqual(testCase, task, taskBefore);
    verifyEqual(testCase, semanticStrokeSet.summary.semantic_stroke_count, 2);
    verifyEqual(testCase, [semanticStrokeSet.strokes.stroke_id], [1, 2]);
    verifyEqual(testCase, ...
        {semanticStrokeSet.strokes.source_indices}, {[1, 2], [4, 5]});
    verifyEqual(testCase, ...
        [semanticStrokeSet.strokes.semantic_stroke_index], [1, 2]);
end
