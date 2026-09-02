function compiled = compile_styled_banter_motion_plan(styledMotionPlan, robotContext, timingConfig)
%COMPILE_STYLED_BANTER_MOTION_PLAN Task 6 timing-only wrapper over Task 5.
%   Style never changes a named pose or joint angle.  It validates the
%   derived duration-only plan, then delegates all numerical work and safety
%   evidence to the unchanged Task 5 Legacy5 compiler.

    if nargin < 2 || isempty(robotContext)
        robotContext = load_robot_context('legacy5');
    end
    if nargin < 3 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config(robotContext);
    end
    inputSnapshot = styledMotionPlan;
    validate_styled_motion_plan(styledMotionPlan);

    sourcePlan = styledMotionPlan.source_motion_plan;
    derivedPlan = sourcePlan;
    derivedPlan.motion_steps = styledMotionPlan.motion_steps;
    derivedPlan.available_t_ms = double(styledMotionPlan.available_t_ms);
    [derivedPlan, timingResolution, toppraReference] = resolve_fist_timing_band( ...
        derivedPlan, styledMotionPlan, robotContext, timingConfig);
    nominalCompiled = compile_banter_motion_plan(derivedPlan, robotContext, timingConfig);
    if toppraReference.enabled
        [retimedContinuous, applied, fallbackReason] = apply_fist_toppra_reference( ...
            nominalCompiled, toppraReference, timingResolution.band_factor, ...
            timingConfig, robotContext);
        if ~applied
            error('MonoTeach:BanterToppraApplication', ...
                'Validated TOPP-RA reference could not be applied: %s.', fallbackReason);
        end
        nominalCompiled.continuous_trajectory = retimedContinuous;
        nominalCompiled.summary = summarize_compiled_toppra_motion( ...
            nominalCompiled.summary, retimedContinuous);
    end

    if ~isequaln(styledMotionPlan, inputSnapshot)
        error('MonoTeach:BanterStyledMotionPlanMutation', ...
            'compile_styled_banter_motion_plan must not modify its input.');
    end
    summary = nominalCompiled.summary;
    summary.requested_styled_duration_s = sum(double([styledMotionPlan.motion_steps.duration_ms])) / 1000.0;
    summary.resolved_timing_duration_s = sum(double([derivedPlan.motion_steps.duration_ms])) / 1000.0;
    summary.source_nominal_duration_s = sum(double([sourcePlan.motion_steps.duration_ms])) / 1000.0;
    summary.style_is_timing_only = true;
    summary.style_timing_resolution = timingResolution;
    compiled = nominalCompiled;
    compiled.artifact_type = 'CompiledStyledBanterMotion';
    compiled.schema_version = 'banter_compiled_styled_motion_v1';
    compiled.source_motion_plan = sourcePlan;
    compiled.styled_motion_plan = styledMotionPlan;
    compiled.resolved_motion_plan = derivedPlan;
    compiled.style = styledMotionPlan.style;
    compiled.summary = summary;
end

function [resolvedPlan, resolution, toppraReference] = resolve_fist_timing_band( ...
        requestedPlan, styledPlan, robotContext, timingConfig)
% Resolve FIST timing through Stage 3's existing TOPP-RA diagnostic.
% The Task 5 quintic compiler is used solely to provide that diagnostic's
% existing piecewise-geometric path input.  Its safety-stretched duration is
% never selected as a FIST timing baseline or a fallback trajectory.
    resolution = struct( ...
        'method', 'REQUESTED_STYLE_ONLY', ...
        'band_factor', 1.0, ...
        'fastest_safe_nonhold_duration_s', NaN, ...
        'preserved_hold_duration_s', NaN, ...
        'probe_scale', NaN, ...
        'limit_source', robotContext.limit_source, ...
        'retiming_method', 'NOT_APPLICABLE', ...
        'baseline_duration_s', NaN, ...
        'toppra_duration_s', NaN, ...
        'toppra_sample_count', NaN, ...
        'toppra_path_joint_deviation_rad', NaN, ...
        'toppra_path_fk_deviation_m', NaN, ...
        'fallback_reason', 'NOT_FIST_COUNTER');
    toppraReference = empty_toppra_reference();
    resolvedPlan = requestedPlan;
    if ~strcmp(string(styledPlan.behavior_name), "FIST_COUNTER")
        return;
    end

    % This compiled result supplies only the existing quintic pp geometry
    % expected by diagnose_contopptraj_retiming.  It is not an executable
    % FIST timing fallback: the TOPP-RA result below always replaces movement.
    geometryCompiled = compile_banter_motion_plan(requestedPlan, robotContext, timingConfig);
    [toppraReference, toppraLowerBoundsS] = evaluate_fist_toppra_reference( ...
        requestedPlan, geometryCompiled, robotContext);
    bandFactor = fist_timing_band_factor(string(styledPlan.variant));
    for index = 1:numel(resolvedPlan.motion_steps)
        if ~strcmp(string(resolvedPlan.motion_steps(index).kind), "HOLD")
            resolvedPlan.motion_steps(index).duration_ms = ...
                toppraLowerBoundsS(index) * bandFactor * 1000.0;
        end
    end
    holdDurationS = sum(double([styledPlan.motion_steps( ...
        strcmp(string({styledPlan.motion_steps.kind}), "HOLD")).duration_ms])) / 1000.0;
    resolution = struct( ...
        'method', 'FIST_TOPPRA_TIMING_V1', ...
        'band_factor', bandFactor, ...
        'fastest_safe_nonhold_duration_s', sum(toppraLowerBoundsS), ...
        'preserved_hold_duration_s', holdDurationS, ...
        'probe_scale', NaN, ...
        'limit_source', robotContext.limit_source, ...
        'retiming_method', 'MATLAB_CONTOPPTRAJ_TOPPRA', ...
        'baseline_duration_s', toppraReference.baseline_duration_s, ...
        'toppra_duration_s', toppraReference.toppra_duration_s, ...
        'toppra_sample_count', toppraReference.sample_count, ...
        'toppra_path_joint_deviation_rad', toppraReference.path_joint_deviation_rad, ...
        'toppra_path_fk_deviation_m', toppraReference.path_fk_deviation_m, ...
        'fallback_reason', '');
end

function [reference, toppraLowerBoundsS] = evaluate_fist_toppra_reference( ...
        requestedPlan, geometryCompiled, robotContext)
% Borrow Stage 3's existing diagnostic; never alter its formal artifact.
    reference = empty_toppra_reference();
    toppraLowerBoundsS = NaN(1, numel(requestedPlan.motion_steps));
    movement = ~strcmp(string({geometryCompiled.segment_groups.kind}), "HOLD");
    baseline = geometryCompiled.continuous_trajectory;
    baseline.segments = baseline.segments(movement);
    groups = geometryCompiled.segment_groups(movement);
    timed = struct('segments', baseline.segments);
    % Start with the smallest useful diagnostic and increase only when the
    % existing evidence guards show that its discrete path is too coarse.
    % This remains one TOPP-RA resolver, not a timing fallback.
    diagnostic = struct();
    sampleCount = NaN;
    failureReason = 'TOPPRA_NOT_EVALUATED';
    for candidateSampleCount = [100, 200, 500, 1000]
        try
            candidate = diagnose_contopptraj_retiming( ...
                timed, baseline, robotContext, candidateSampleCount);
        catch exception
            failureReason = ['TOPPRA_DIAGNOSTIC_ERROR:' exception.identifier];
            break;
        end
        if ~candidate.available
            failureReason = char(string(candidate.unavailable_reason));
            break;
        end
        if ~candidate.summary.velocity_limits_satisfied || ...
                ~candidate.summary.acceleration_limits_satisfied || ...
                ~candidate.summary.joint_limits_satisfied
            failureReason = 'TOPPRA_SAFETY_EVIDENCE_FAILED';
            break;
        end
        if ~isfinite(candidate.summary.max_joint_space_geometric_deviation_rad) || ...
                ~isfinite(candidate.summary.max_geometric_fk_distance_m) || ...
                candidate.summary.max_joint_space_geometric_deviation_rad > 1.0e-4 || ...
                candidate.summary.max_geometric_fk_distance_m > 1.0e-5
            failureReason = 'TOPPRA_PATH_DEVIATION_EXCEEDED';
            continue;
        end
        diagnostic = candidate;
        sampleCount = candidateSampleCount;
        break;
    end
    if isnan(sampleCount)
        throw_toppra_required(failureReason);
    end
    baselineDurationS = sum([baseline.segments.duration_s]);
    toppraDurationS = diagnostic.summary.total_duration_s;
    if ~(isfinite(toppraDurationS) && toppraDurationS > 0.0)
        throw_toppra_required('TOPPRA_DURATION_INVALID');
    end
    for index = 1:numel(groups)
        sourceSteps = groups(index).source_step_indices;
        requestedGroupS = sum(double([requestedPlan.motion_steps(sourceSteps).duration_ms])) / 1000.0;
        if isempty(sourceSteps) || requestedGroupS <= 0.0
            throw_toppra_required('TOPPRA_SOURCE_STEP_ALLOCATION_FAILED');
        end
        toppraLowerBoundsS(sourceSteps) = ...
            double([requestedPlan.motion_steps(sourceSteps).duration_ms]) / 1000.0 * ...
            (diagnostic.segments(index).duration_s / requestedGroupS);
    end
    nonhold = ~strcmp(string({requestedPlan.motion_steps.kind}), "HOLD");
    if any(~isfinite(toppraLowerBoundsS(nonhold))) || ...
            any(toppraLowerBoundsS(nonhold) <= 0.0)
        throw_toppra_required('TOPPRA_SOURCE_STEP_ALLOCATION_FAILED');
    end
    toppraLowerBoundsS(~nonhold) = 0.0;
    reference.enabled = true;
    reference.diagnostic = diagnostic;
    reference.groups = groups;
    reference.baseline_duration_s = baselineDurationS;
    reference.toppra_duration_s = toppraDurationS;
    reference.sample_count = sampleCount;
    reference.path_joint_deviation_rad = diagnostic.summary.max_joint_space_geometric_deviation_rad;
    reference.path_fk_deviation_m = diagnostic.summary.max_geometric_fk_distance_m;
    reference.fallback_reason = '';
end

function throw_toppra_required(reason)
    error('MonoTeach:BanterToppraRequired', ...
        ['FIST timing requires the existing Stage 3 contopptraj/TOPP-RA ' ...
        'diagnostic and valid safety/path evidence (%s).'], reason);
end

function reference = empty_toppra_reference()
    reference = struct( ...
        'enabled', false, 'diagnostic', struct(), 'groups', struct([]), ...
        'baseline_duration_s', NaN, 'toppra_duration_s', NaN, ...
        'sample_count', NaN, ...
        'path_joint_deviation_rad', NaN, 'path_fk_deviation_m', NaN, ...
        'fallback_reason', 'TOPPRA_NOT_EVALUATED');
end

function [continuous, applied, reason] = apply_fist_toppra_reference( ...
        compiled, reference, bandFactor, timingConfig, robotContext)
% Build a Task 6 derived trajectory from the existing TOPP-RA diagnostic.
    continuous = compiled.continuous_trajectory;
    applied = false;
    reason = '';
    movementIndices = find(~strcmp(string({compiled.segment_groups.kind}), "HOLD"));
    if numel(movementIndices) ~= numel(reference.groups) || ...
            numel(reference.diagnostic.segments) ~= numel(reference.groups)
        reason = 'MOVEMENT_GROUP_COUNT_MISMATCH';
        return;
    end
    for index = 1:numel(movementIndices)
        group = compiled.segment_groups(movementIndices(index));
        referenceGroup = reference.groups(index);
        if ~isequal(group.source_step_indices, referenceGroup.source_step_indices)
            reason = 'MOVEMENT_GROUP_PROVENANCE_MISMATCH';
            return;
        end
        shell = continuous.segments(movementIndices(index));
        diagnostic = reference.diagnostic.segments(index);
        if norm(shell.q_rad(1, :) - diagnostic.q_rad(1, :), inf) > 1.0e-10 || ...
                norm(shell.q_rad(end, :) - diagnostic.q_rad(end, :), inf) > 1.0e-10
            reason = 'TOPPRA_ENDPOINT_MISMATCH';
            return;
        end
        continuous.segments(movementIndices(index)) = make_toppra_segment( ...
            shell, diagnostic, bandFactor, timingConfig, robotContext);
    end
    continuous.generation_method = 'quintic_path_toppra_fist_timing_v1';
    continuous.time_allocation_optimized = true;
    continuous.summary = summarize_toppra_continuous(continuous.segments, timingConfig);
    applied = true;
end

function segment = make_toppra_segment(shell, diagnostic, bandFactor, config, context)
    segment = shell;
    localT = (diagnostic.t_s(:) - diagnostic.t_s(1)) * bandFactor;
    qd = diagnostic.qd_rad_s / bandFactor;
    qdd = diagnostic.qdd_rad_s2 / (bandFactor ^ 2);
    qddd = numerical_jerk(qdd, localT);
    peakVelocity = max(abs(qd), [], 1);
    peakAcceleration = max(abs(qdd), [], 1);
    margin = min(diagnostic.q_rad - context.joint_limits(:, 1)', ...
        context.joint_limits(:, 2)' - diagnostic.q_rad);
    segment.t_s = localT;
    segment.q_rad = diagnostic.q_rad;
    segment.qd_rad_s = qd;
    segment.qdd_rad_s2 = qdd;
    segment.qddd_rad_s3 = qddd;
    segment.waypoint_t_s = [0.0; localT(end)];
    segment.waypoint_qd_rad_s = [qd(1, :); qd(end, :)];
    segment.waypoint_qdd_rad_s2 = [qdd(1, :); qdd(end, :)];
    segment.velocity_boundary_condition_rad_s = segment.waypoint_qd_rad_s;
    segment.acceleration_boundary_condition_rad_s2 = segment.waypoint_qdd_rad_s2;
    segment.piecewise_polynomial = struct( ...
        'retiming_method', 'matlab_contopptraj_toppra', ...
        'path_source', 'existing_quintic_piecewise_polynomial');
    segment.initial_duration_s = localT(end);
    segment.duration_s = localT(end);
    segment.time_stretch_iterations = 0;
    segment.time_stretch_factor = 1.0;
    segment.was_time_stretched = false;
    segment.initial_velocity_limit_ratio = max(peakVelocity ./ config.max_velocity_rad_s);
    segment.initial_acceleration_limit_ratio = max(peakAcceleration ./ config.max_acceleration_rad_s2);
    segment.initial_peak_velocity_rad_s = peakVelocity;
    segment.initial_peak_acceleration_rad_s2 = peakAcceleration;
    segment.max_observed_abs_velocity_rad_s = peakVelocity;
    segment.max_observed_abs_acceleration_rad_s2 = peakAcceleration;
    segment.max_observed_abs_jerk_rad_s3 = max(abs(qddd), [], 1);
    segment.velocity_limits_satisfied = all(peakVelocity <= config.max_velocity_rad_s + 1.0e-10);
    segment.acceleration_limits_satisfied = all(peakAcceleration <= config.max_acceleration_rad_s2 + 1.0e-10);
    segment.within_joint_limits = all(diagnostic.q_rad >= context.joint_limits(:, 1)' - 1.0e-10 & ...
        diagnostic.q_rad <= context.joint_limits(:, 2)' + 1.0e-10, 2);
    segment.joint_limit_margin = margin;
    segment.minimum_joint_limit_margin = min(margin, [], 'all');
    segment.joint_limits_satisfied = all(segment.within_joint_limits);
    segment.all_safety_checks_satisfied = segment.velocity_limits_satisfied && ...
        segment.acceleration_limits_satisfied && segment.joint_limits_satisfied && ...
        all(isfinite(qd), 'all') && all(isfinite(qdd), 'all') && all(isfinite(qddd), 'all');
end

function jerk = numerical_jerk(acceleration, time)
    jerk = zeros(size(acceleration));
    if numel(time) < 2
        return;
    end
    jerk(1, :) = (acceleration(2, :) - acceleration(1, :)) / (time(2) - time(1));
    jerk(end, :) = (acceleration(end, :) - acceleration(end - 1, :)) / ...
        (time(end) - time(end - 1));
    for index = 2:numel(time) - 1
        jerk(index, :) = (acceleration(index + 1, :) - acceleration(index - 1, :)) / ...
            (time(index + 1) - time(index - 1));
    end
end

function summary = summarize_toppra_continuous(segments, config)
    summary = struct( ...
        'continuous_segment_count', numel(segments), ...
        'continuous_sample_count', sum(arrayfun(@(item) numel(item.t_s), segments)), ...
        'total_independent_duration_s', sum([segments.duration_s]), ...
        'max_velocity_rad_s', config.max_velocity_rad_s, ...
        'max_acceleration_rad_s2', config.max_acceleration_rad_s2, ...
        'time_allocation_optimized', true, ...
        'max_observed_abs_velocity_rad_s', max(vertcat(segments.max_observed_abs_velocity_rad_s), [], 1), ...
        'max_observed_abs_acceleration_rad_s2', max(vertcat(segments.max_observed_abs_acceleration_rad_s2), [], 1), ...
        'max_observed_abs_jerk_rad_s3', max(vertcat(segments.max_observed_abs_jerk_rad_s3), [], 1), ...
        'all_safety_checks_satisfied', all([segments.all_safety_checks_satisfied]), ...
        'all_joint_limits_satisfied', all([segments.joint_limits_satisfied]), ...
        'max_joint_overshoot_rad', max(arrayfun(@(item) ...
            item.overshoot_evidence.max_overshoot_magnitude_rad, segments)));
end

function summary = summarize_compiled_toppra_motion(summary, continuous)
    segments = continuous.segments;
    evidence = [segments.overshoot_evidence];
    summary.execution_duration_s = sum([segments.duration_s]);
    summary.minimum_joint_limit_margin_rad = min([segments.minimum_joint_limit_margin]);
    summary.max_observed_abs_velocity_rad_s = max(vertcat(segments.max_observed_abs_velocity_rad_s), [], 1);
    summary.max_observed_abs_acceleration_rad_s2 = max(vertcat(segments.max_observed_abs_acceleration_rad_s2), [], 1);
    summary.all_safety_checks_satisfied = all([segments.all_safety_checks_satisfied]);
    summary.has_joint_overshoot = any([evidence.has_overshoot]);
end

function factor = fist_timing_band_factor(variant)
% Discrete semantic bands, intentionally not a continuous intensity control.
    switch string(variant)
        case "FORCEFUL"
            factor = 1.00;
        case "FIRM"
            factor = 1.15;
        case "DEFAULT"
            factor = 1.35;
        otherwise
            error('MonoTeach:BanterTimingResolution', ...
                'FIST timing resolution requires DEFAULT, FIRM, or FORCEFUL.');
    end
end

function validate_styled_motion_plan(plan)
    required = {'schema_version', 'styled_plan_id', 'source_motion_plan', ...
        'macro_name', 'behavior_name', 'variant', 'style', 'motion_steps', 'available_t_ms'};
    if ~isstruct(plan) || ~isscalar(plan) || ~all(isfield(plan, required)) || ...
            ~strcmp(string(plan.schema_version), "banter_styled_motion_plan_v1") || ...
            ~isscalar(plan.styled_plan_id) || ~isfinite(double(plan.styled_plan_id)) || ...
            double(plan.styled_plan_id) < 1 || double(plan.styled_plan_id) ~= floor(double(plan.styled_plan_id)) || ...
            ~isstruct(plan.source_motion_plan) || ~isscalar(plan.source_motion_plan) || ...
            ~isstruct(plan.style) || ~isscalar(plan.style) || ...
            ~isstruct(plan.motion_steps) || isempty(plan.motion_steps) || ...
            ~isscalar(plan.available_t_ms) || ~isfinite(double(plan.available_t_ms))
        error('MonoTeach:BanterInvalidStyledMotionPlan', ...
            'Task 6 requires one valid banter_styled_motion_plan_v1 struct.');
    end
    source = plan.source_motion_plan;
    if ~all(isfield(source, {'schema_version', 'source_behavior', 'macro_name', 'variant', 'motion_steps', 'available_t_ms'})) || ...
            ~isstruct(source.source_behavior) || ...
            ~all(isfield(source.source_behavior, {'behavior_name', 'variant'})) || ...
            ~strcmp(string(plan.macro_name), string(source.macro_name)) || ...
            ~strcmp(string(plan.behavior_name), string(source.source_behavior.behavior_name)) || ...
            ~strcmp(string(plan.variant), string(source.source_behavior.variant))
        error('MonoTeach:BanterStyledProvenance', ...
            'Styled MotionPlan must retain its source MotionPlan behavior, variant, and Macro identity.');
    end
    style = plan.style;
    requiredStyle = {'schema_version', 'variant', 'move_duration_scale', ...
        'hold_duration_scale', 'return_duration_scale'};
    if ~all(isfield(style, requiredStyle)) || isfield(style, 'pose_excursion_scale') || ...
            ~strcmp(string(style.schema_version), "banter_motion_style_v1") || ...
            ~strcmp(string(style.variant), string(plan.variant))
        error('MonoTeach:BanterInvalidMotionStyle', ...
            'Task 6 style must be timing-only and match the source variant.');
    end
    scales = [double(style.move_duration_scale), double(style.hold_duration_scale), ...
        double(style.return_duration_scale)];
    if any(~isfinite(scales)) || any(scales <= 0)
        error('MonoTeach:BanterInvalidMotionStyle', ...
            'Task 6 timing scales must be finite and positive.');
    end
    if double(plan.available_t_ms) < double(source.available_t_ms)
        error('MonoTeach:BanterStyledAvailability', ...
            'Styled MotionPlan availability must not precede source MotionPlan availability.');
    end
    if numel(plan.motion_steps) ~= numel(source.motion_steps)
        error('MonoTeach:BanterStyledTopology', ...
            'Task 6 may not change the source MotionPlan step count.');
    end
    for index = 1:numel(source.motion_steps)
        original = source.motion_steps(index); styled = plan.motion_steps(index);
        if ~all(isfield(styled, {'kind', 'target_pose_name', 'duration_ms'})) || ...
                ~strcmp(string(styled.kind), string(original.kind)) || ...
                ~strcmp(string(styled.target_pose_name), string(original.target_pose_name))
            error('MonoTeach:BanterStyledTopology', ...
                'Task 6 may not change source step kinds or pose names.');
        end
        scale = duration_scale_for_kind(string(original.kind), scales);
        expected = double(original.duration_ms) * scale;
        if ~isfinite(double(styled.duration_ms)) || double(styled.duration_ms) <= 0 || ...
                abs(double(styled.duration_ms) - expected) > 1.0e-9 * max(1.0, expected)
            error('MonoTeach:BanterStyledDuration', ...
                'Every styled duration must equal the selected fixed timing scale.');
        end
    end
end

function scale = duration_scale_for_kind(kind, scales)
    if kind == "MOVE_POSE"
        scale = scales(1);
    elseif kind == "HOLD"
        scale = scales(2);
    elseif kind == "RETURN_NEUTRAL"
        scale = scales(3);
    else
        error('MonoTeach:BanterStyledTopology', 'Unknown Task 6 MotionStep kind %s.', char(kind));
    end
end
