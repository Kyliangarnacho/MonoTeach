function tests = test_banter_task6_motion
%TEST_BANTER_TASK6_MOTION Timing-only discrete styling through the Legacy5 compiler.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(repoRoot, 'stage3')));
    addpath(fullfile(repoRoot, 'stage1'));
    addpath(fullfile(repoRoot, 'banter', 'matlab'));
    testCase.TestData.context = load_robot_context('legacy5');
end

function testFistTimingStylesCompileSafelyWithoutChangingPoseEndpoints(testCase)
    context = testCase.TestData.context;
    defaultPlan = styled_fist_plan(1, 'DEFAULT', 1.00, 1.00, 1.00);
    firmPlan = styled_fist_plan(2, 'FIRM', 0.90, 1.00, 0.95);
    forcefulPlan = styled_fist_plan(3, 'FORCEFUL', 0.78, 1.20, 0.90);
    defaultCompiled = compile_styled_banter_motion_plan(defaultPlan, context);
    firmCompiled = compile_styled_banter_motion_plan(firmPlan, context);
    forcefulCompiled = compile_styled_banter_motion_plan(forcefulPlan, context);
    compiled = [defaultCompiled, firmCompiled, forcefulCompiled];
    for index = 1:numel(compiled)
        segments = compiled(index).continuous_trajectory.segments;
        overshootEvidence = [segments.overshoot_evidence];
        verifyTrue(testCase, all([segments.all_safety_checks_satisfied]));
        verifyFalse(testCase, any([overshootEvidence.has_overshoot]));
        verifyEqual(testCase, segments(end).q_rad(end, :), context.home_q, 'AbsTol', 1.0e-12);
        verifyTrue(testCase, compiled(index).summary.style_is_timing_only);
        verifyEqual(testCase, compiled(index).summary.style_timing_resolution.method, ...
            'FIST_TOPPRA_TIMING_V1');
        verifyLessThanOrEqual(testCase, max([segments.time_stretch_factor]), 1.0 + 1.0e-8);
        resolution = compiled(index).summary.style_timing_resolution;
        verifyEqual(testCase, exist('contopptraj', 'file'), 2, ...
            'Task 6 FIST timing requires the existing Stage 3 TOPP-RA capability.');
        verifyEqual(testCase, resolution.retiming_method, 'MATLAB_CONTOPPTRAJ_TOPPRA');
        verifyEmpty(testCase, resolution.fallback_reason);
        verifyEqual(testCase, resolution.toppra_sample_count, 500);
        verifyGreaterThan(testCase, resolution.baseline_duration_s, resolution.toppra_duration_s);
        verifyTrue(testCase, compiled(index).continuous_trajectory.time_allocation_optimized);
    end
    verifyGreaterThan(testCase, defaultCompiled.summary.requested_styled_duration_s, firmCompiled.summary.requested_styled_duration_s);
    verifyGreaterThan(testCase, firmCompiled.summary.requested_styled_duration_s, forcefulCompiled.summary.requested_styled_duration_s);
    verifyGreaterThan(testCase, defaultCompiled.summary.execution_duration_s, firmCompiled.summary.execution_duration_s);
    verifyGreaterThan(testCase, firmCompiled.summary.execution_duration_s, forcefulCompiled.summary.execution_duration_s);
    verifyGreaterThan(testCase, defaultCompiled.summary.resolved_timing_duration_s, firmCompiled.summary.resolved_timing_duration_s);
    verifyGreaterThan(testCase, firmCompiled.summary.resolved_timing_duration_s, forcefulCompiled.summary.resolved_timing_duration_s);
    defaultSegments = defaultCompiled.continuous_trajectory.segments;
    forcefulSegments = forcefulCompiled.continuous_trajectory.segments;
    verifyEqual(testCase, defaultCompiled.segmentation_method, 'HOLD_BOUNDARY_RUNS_V1');
    verifyEqual(testCase, string({defaultCompiled.segment_groups.kind}), ...
        string({forcefulCompiled.segment_groups.kind}));
    for index = 1:numel(defaultSegments)
        verifyEqual(testCase, defaultSegments(index).q_rad(1, :), forcefulSegments(index).q_rad(1, :), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, defaultSegments(index).q_rad(end, :), forcefulSegments(index).q_rad(end, :), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, forcefulSegments(index).qd_rad_s(1, :), zeros(1, context.dof), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, forcefulSegments(index).qd_rad_s(end, :), zeros(1, context.dof), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, forcefulSegments(index).qdd_rad_s2(1, :), zeros(1, context.dof), 'AbsTol', 1.0e-12);
        verifyEqual(testCase, forcefulSegments(index).qdd_rad_s2(end, :), zeros(1, context.dof), 'AbsTol', 1.0e-12);
    end
end

function testCompilerRejectsPoseScalingAndTopologyChanges(testCase)
    context = testCase.TestData.context;
    invalidStyle = styled_fist_plan(1, 'FIRM', 0.90, 1.00, 0.95);
    invalidStyle.style.pose_excursion_scale = 0.90;
    verifyError(testCase, @() compile_styled_banter_motion_plan(invalidStyle, context), ...
        'MonoTeach:BanterInvalidMotionStyle');
    invalidTopology = styled_fist_plan(2, 'FIRM', 0.90, 1.00, 0.95);
    invalidTopology.motion_steps(1).target_pose_name = 'AIM';
    verifyError(testCase, @() compile_styled_banter_motion_plan(invalidTopology, context), ...
        'MonoTeach:BanterStyledTopology');
end

function testStyleGalleryUsesPersistentVisualizer(testCase)
    source = fileread(which('demo_banter_motion_styles'));
    verifyTrue(testCase, contains(source, 'live_legacy5_visualizer(''create'''));
    verifyTrue(testCase, contains(source, 'live_legacy5_visualizer(''update'''));
    verifyFalse(testCase, contains(source, 'show(context.model'));
    verifyTrue(testCase, contains(source, 'resolved %.2fs'));
    verifyTrue(testCase, contains(source, 'render_sample_indices'));
    verifyTrue(testCase, contains(source, 'renderRateHz = 25.0'));
    verifyTrue(testCase, contains(source, '"HOLD"'));
    verifyTrue(testCase, contains(source, 'desired = (segment.t_s(sampleIndex) - firstT) / playbackRate'));
end

function testFistTimingHasNoQuinticDurationFallback(testCase)
    source = fileread(which('compile_styled_banter_motion_plan'));
    verifyTrue(testCase, contains(source, 'MonoTeach:BanterToppraRequired'));
    verifyFalse(testCase, contains(source, 'fastest_safe_nonhold_step_durations'));
    verifyFalse(testCase, contains(source, 'QUINTIC_SAFE_DURATION'));
end

function plan = styled_fist_plan(planId, variant, moveScale, holdScale, returnScale)
    sourceSteps = [ ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'RECOIL', 'duration_ms', 300.0), ...
        struct('kind', 'MOVE_POSE', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'HOLD', 'target_pose_name', 'JAB_EXTEND', 'duration_ms', 250.0), ...
        struct('kind', 'RETURN_NEUTRAL', 'target_pose_name', 'NEUTRAL', 'duration_ms', 500.0)];
    styledSteps = sourceSteps;
    for index = 1:numel(styledSteps)
        if strcmp(styledSteps(index).kind, 'MOVE_POSE')
            scale = moveScale;
        elseif strcmp(styledSteps(index).kind, 'HOLD')
            scale = holdScale;
        else
            scale = returnScale;
        end
        styledSteps(index).duration_ms = styledSteps(index).duration_ms * scale;
    end
    sourceBehavior = struct('behavior_id', planId, 'behavior_name', 'FIST_COUNTER', ...
        'variant', variant, 'source_token_event_ids', planId, 'source_phrase_ids', zeros(1, 0));
    source = struct('schema_version', 'banter_motion_plan_v1', 'plan_id', planId, ...
        'source_behavior', sourceBehavior, 'macro_name', 'COUNTER_JAB', 'variant', variant, ...
        'motion_steps', sourceSteps, 'available_t_ms', 1000.0);
    style = struct('schema_version', 'banter_motion_style_v1', 'variant', variant, ...
        'move_duration_scale', moveScale, 'hold_duration_scale', holdScale, ...
        'return_duration_scale', returnScale);
    plan = struct('schema_version', 'banter_styled_motion_plan_v1', 'styled_plan_id', planId, ...
        'source_motion_plan', source, 'macro_name', 'COUNTER_JAB', ...
        'behavior_name', 'FIST_COUNTER', 'variant', variant, 'style', style, ...
        'motion_steps', styledSteps, 'available_t_ms', 1100.0);
end
