function library = load_banter_execution_library(fixturePath, robotContext, timingConfig)
%LOAD_BANTER_EXECUTION_LIBRARY Precompile Task 6 plans for one live session.
%   The cache is process-local only.  Incoming plans must structurally match
%   their source fixture before an already-validated q(t) trajectory is used.

    if nargin < 1 || isempty(fixturePath)
        error('MonoTeach:BanterFixtureRequired', 'Task 7 requires a Task 6 styled-plan fixture.');
    end
    if nargin < 2 || isempty(robotContext), robotContext = load_robot_context('legacy5'); end
    if nargin < 3, timingConfig = []; end
    payload = jsondecode(fileread(fixturePath));
    if ~isfield(payload, 'plans') || ~isstruct(payload.plans)
        error('MonoTeach:BanterFixture', 'Fixture must contain a plans struct array.');
    end
    plans = payload.plans;
    entries = repmat(struct('key', '', 'source_plan', struct(), 'compiled', struct()), 1, numel(plans));
    keys = strings(1, numel(plans));
    for index = 1:numel(plans)
        plan = plans(index);
        key = char(string(plan.behavior_name) + ":" + string(plan.variant));
        if any(keys(1:index-1) == string(key))
            error('MonoTeach:BanterDuplicateExecutionKey', 'Fixture repeats execution key %s.', key);
        end
        compiled = compile_styled_banter_motion_plan(plan, robotContext, timingConfig);
        assert_compiled_banter_execution_safe(compiled, robotContext);
        entries(index) = struct('key', key, 'source_plan', plan, 'compiled', compiled);
        keys(index) = string(key);
        fprintf('[Task7] precompiled %s (%.3f s)\n', key, compiled.summary.execution_duration_s);
    end
    library = struct('artifact_type', 'BanterExecutionLibrary', ...
        'schema_version', 'banter_execution_v1', 'fixture_path', char(fixturePath), ...
        'robot_context', robotContext, 'entries', entries);
end

function assert_compiled_banter_execution_safe(compiled, context)
    if ~isfield(compiled, 'continuous_trajectory') || ~isfield(compiled.continuous_trajectory, 'segments')
        error('MonoTeach:BanterExecutionCompile', 'Compiled motion has no continuous trajectory.');
    end
    segments = compiled.continuous_trajectory.segments;
    for index = 1:numel(segments)
        segment = segments(index);
        if ~all(isfinite(segment.q_rad), 'all') || ~all(isfinite(segment.t_s), 'all') || ...
                ~all(segment.within_joint_limits, 'all') || ~all(segment.velocity_limits_satisfied, 'all') || ...
                ~all(segment.acceleration_limits_satisfied, 'all')
            error('MonoTeach:BanterExecutionUnsafe', 'Compiled segment %d lacks valid Task 5/6 safety evidence.', index);
        end
    end
    finalQ = segments(end).q_rad(end, :);
    if norm(finalQ - context.home_q, inf) > 1.0e-8
        error('MonoTeach:BanterExecutionNeutral', 'Every Task 7 action must end at Legacy5 neutral/home.');
    end
end
