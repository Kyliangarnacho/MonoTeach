function tests = test_banter_task5_motion
%TEST_BANTER_TASK5_MOTION Offline Legacy5 compiler coverage for Task 5.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(repoRoot, 'stage3')));
    addpath(fullfile(repoRoot, 'stage1'));
    addpath(fullfile(repoRoot, 'banter', 'matlab'));
    testCase.TestData.context = load_robot_context('legacy5');
end

function testEveryNamedPoseCompilesWithinExistingQuinticSafetyLimits(testCase)
    context = testCase.TestData.context;
    poses = default_legacy5_banter_pose_library(context);
    names = fieldnames(poses);
    planId = 1;
    for index = 1:numel(names)
        poseName = names{index};
        if strcmp(poseName, 'NEUTRAL'), continue, end
        plan = one_pose_plan(planId, poseName);
        snapshot = plan;
        compiled = compile_banter_motion_plan(plan, context);
        segments = compiled.continuous_trajectory.segments;
        overshootEvidence = [segments.overshoot_evidence];
        verifyEqual(testCase, plan, snapshot);
        verifyTrue(testCase, all([segments.all_safety_checks_satisfied]));
        verifyFalse(testCase, any([overshootEvidence.has_overshoot]));
        verifyGreaterThanOrEqual(testCase, min([segments.minimum_joint_limit_margin]), -1.0e-10);
        verifyLessThanOrEqual(testCase, max(vertcat(segments.max_observed_abs_velocity_rad_s), [], 1), ...
            context.velocity_limits + 1.0e-9);
        verifyLessThanOrEqual(testCase, max(vertcat(segments.max_observed_abs_acceleration_rad_s2), [], 1), ...
            context.acceleration_limits + 1.0e-9);
        verifyEqual(testCase, segments(1).q_rad(1, :), context.home_q, 'AbsTol', 1.0e-12);
        verifyEqual(testCase, segments(end).q_rad(end, :), context.home_q, 'AbsTol', 1.0e-12);
        planId = planId + 1;
    end
end

function testCompilerRejectsUnknownPoseAndInvalidReturn(testCase)
    context = testCase.TestData.context;
    unknown = one_pose_plan(1, 'NO_SUCH_POSE');
    verifyError(testCase, @() compile_banter_motion_plan(unknown, context), ...
        'MonoTeach:BanterUnknownPose');
    invalid = one_pose_plan(2, 'AIM');
    invalid.motion_steps(end).target_pose_name = 'AIM';
    verifyError(testCase, @() compile_banter_motion_plan(invalid, context), ...
        'MonoTeach:BanterInvalidMotionPlanReturn');
end

function testPointJabGroupsMovesAroundAnExplicitStationaryHold(testCase)
    context = testCase.TestData.context;
    steps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'AIM', 'duration_ms', 300.0), ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 280.0), ...
        struct('kind', 'HOLD', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 450.0)];
    plan = motion_plan(99, 'POINT_JAB', steps);
    compiled = compile_banter_motion_plan(plan, context);
    segments = compiled.continuous_trajectory.segments;
    overshootEvidence = [segments.overshoot_evidence];
    groups = compiled.segment_groups;
    verifyEqual(testCase, compiled.segmentation_method, 'HOLD_BOUNDARY_RUNS_V1');
    verifyEqual(testCase, numel(segments), 3);
    verifyEqual(testCase, numel(groups), 3);
    verifyEqual(testCase, string({groups.kind}), ["MOVE_RUN", "HOLD", "MOVE_RUN"]);
    verifyEqual(testCase, groups(1).source_step_indices, [1, 2]);
    verifyEqual(testCase, groups(2).source_step_indices, 3);
    verifyEqual(testCase, groups(3).source_step_indices, 4);
    verifyTrue(testCase, all([segments.all_safety_checks_satisfied]));
    verifyFalse(testCase, any([overshootEvidence.has_overshoot]));
    for index = 1:numel(segments)-1
        verifyEqual(testCase, segments(index).q_rad(end, :), ...
            segments(index + 1).q_rad(1, :), 'AbsTol', 1.0e-12);
    end
    verifyGreaterThan(testCase, norm(segments(1).waypoint_qd_rad_s(2, :), inf), 1.0e-6);
    verifyEqual(testCase, segments(2).q_rad, ...
        repmat(segments(2).q_rad(1, :), size(segments(2).q_rad, 1), 1), ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, segments(1).waypoint_qd_rad_s(end, :), zeros(1, context.dof), ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, segments(2).waypoint_qd_rad_s, zeros(size(segments(2).waypoint_qd_rad_s)), ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, segments(2).waypoint_qdd_rad_s2, zeros(size(segments(2).waypoint_qdd_rad_s2)), ...
        'AbsTol', 1.0e-12);
    verifyEqual(testCase, segments(3).waypoint_qd_rad_s(1, :), zeros(1, context.dof), ...
        'AbsTol', 1.0e-12);
end

function testNoHoldMacroUsesLocalGuardBeforeSplitFallback(testCase)
    context = testCase.TestData.context;
    steps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'WAVE_LEFT', 'duration_ms', 240.0), ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'WAVE_RIGHT', 'duration_ms', 240.0), ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'WAVE_LEFT', 'duration_ms', 240.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 420.0)];
    compiled = compile_banter_motion_plan(motion_plan(100, 'SIDE_WAVE', steps), context);
    segments = compiled.continuous_trajectory.segments;
    groups = compiled.segment_groups;
    verifyEqual(testCase, numel(groups), numel(segments));
    verifyEqual(testCase, string({groups.kind}), repmat("MOVE_RUN", 1, numel(groups)));
    verifyEqual(testCase, numel(segments), 1);
    verifyEqual(testCase, string(groups.segmentation_reason), "BANTER_LOCAL_BOUNDARY_GUARD");
    verifyEqual(testCase, compiled.movement_run_guard_evidence(1).status, 'APPLIED');
    verifyTrue(testCase, all([segments.all_safety_checks_satisfied]));
    overshootEvidence = [segments.overshoot_evidence];
    verifyFalse(testCase, any([overshootEvidence.has_overshoot]));
    verifyEqual(testCase, segments(1).q_rad(1, :), context.home_q, 'AbsTol', 1.0e-12);
    verifyEqual(testCase, segments(end).q_rad(end, :), context.home_q, 'AbsTol', 1.0e-12);
    for index = 1:numel(segments) - 1
        verifyEqual(testCase, segments(index).q_rad(end, :), ...
            segments(index + 1).q_rad(1, :), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, segments(index).waypoint_qd_rad_s(end, :), zeros(1, context.dof), ...
            'AbsTol', 1.0e-12);
        verifyEqual(testCase, segments(index + 1).waypoint_qd_rad_s(1, :), zeros(1, context.dof), ...
            'AbsTol', 1.0e-12);
    end
end

function testFistReversalGuardPreservesOneHoldBoundedMovementRun(testCase)
    context = testCase.TestData.context;
    steps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'RECOIL', 'duration_ms', 300.0), ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'HOLD', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 500.0)];
    compiled = compile_banter_motion_plan(motion_plan(102, 'FIST_COUNTER', steps), context);
    groups = compiled.segment_groups;
    guard = compiled.movement_run_guard_evidence(1);
    segments = compiled.continuous_trajectory.segments;
    verifyEqual(testCase, string({groups.kind}), ["MOVE_RUN", "HOLD", "MOVE_RUN"]);
    verifyEqual(testCase, groups(1).source_step_indices, [1, 2]);
    verifyEqual(testCase, string(groups(1).segmentation_reason), "BANTER_LOCAL_BOUNDARY_GUARD");
    verifyEqual(testCase, guard.status, 'APPLIED');
    verifyTrue(testCase, guard.overshoot_before.has_overshoot);
    verifyFalse(testCase, guard.overshoot_after.has_overshoot);
    verifyNotEmpty(testCase, guard.affected_joint_waypoint_indices);
    verifyTrue(testCase, any(guard.velocity_zeroed_mask, 'all'));
    verifyTrue(testCase, any(guard.acceleration_damping_factor < 1.0, 'all'));
    verifyTrue(testCase, all([segments.all_safety_checks_satisfied]));
    overshootEvidence = [segments.overshoot_evidence];
    verifyFalse(testCase, any([overshootEvidence.has_overshoot]));
end

function testCompilerRejectsHoldThatConcealsAMove(testCase)
    context = testCase.TestData.context;
    steps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'AIM', 'duration_ms', 300.0), ...
        struct('kind', 'HOLD', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 450.0)];
    plan = motion_plan(101, 'INVALID_HOLD', steps);
    verifyError(testCase, @() compile_banter_motion_plan(plan, context), ...
        'MonoTeach:BanterInvalidHoldTarget');
end

function testGalleryUsesPersistentStage3Visualizer(testCase)
    source = fileread(which('demo_banter_motion_library'));
    verifyTrue(testCase, contains(source, 'live_legacy5_visualizer(''create'''));
    verifyTrue(testCase, contains(source, 'live_legacy5_visualizer(''update'''));
    verifyFalse(testCase, contains(source, 'show(context.model'));
end

function plan = one_pose_plan(planId, poseName)
    steps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', poseName, 'duration_ms', 500.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 600.0)];
    plan = motion_plan(planId, 'TEST_MACRO', steps);
end

function plan = motion_plan(planId, macroName, steps)
    plan = struct( ...
        'schema_version', 'banter_motion_plan_v1', ...
        'plan_id', planId, ...
        'source_behavior', struct('behavior_id', planId, ...
            'source_token_event_ids', planId, 'source_phrase_ids', zeros(1, 0)), ...
        'macro_name', macroName, ...
        'variant', 'TEST', ...
        'motion_steps', steps, ...
        'available_t_ms', 1000.0);
end
