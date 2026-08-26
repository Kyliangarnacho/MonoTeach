function run = run_replay_trace_execution(tracePath, config)
%RUN_REPLAY_TRACE_EXECUTION Deterministic virtual-camera to Legacy5 executor.
%
% This is a *single-threaded discrete-event simulation*.  ``tracePath`` was
% made by the Python ReplaySource and contains when source observations and
% causally-finalized Cartesian segments become visible.  At each virtual 100 Hz
% control tick, this function releases only ready segments, solves Position-Only
% IK, appends a q(t) chunk, and samples the active q(t) using controller time.
%
% Important clock separation:
%   source_t_ms      -- human demonstration clock, retained in the trace;
%   available_time   -- earliest replay instant a lookahead segment is known;
%   execution time   -- independent local time used to evaluate q(t).
%
% This does not modify the source JSON, Stage 3.3 modules, or robot model.

    if nargin < 1 || isempty(tracePath)
        error('MonoTeach:ReplayTracePathRequired', ...
            'Pass a JSON trace created by demo_replay_execution_trace.py.');
    end
    trace = load_trace(tracePath);
    robotContext = load_robot_context('legacy5');
    if nargin < 2 || isempty(config)
        config = default_replay_execution_config(robotContext);
    end
    validate_config(config, robotContext);

    [ik, ikState] = make_position_only_ik(robotContext);
    queue = cell(1, 0);
    startup = resolve_startup(trace, config);
    [active, ikState, startup] = build_home_to_start_chunk( ...
        startup, trace.source_events, robotContext, ik, ikState, config);
    nextReadySegment = 1;
    publishedCount = 0;
    execution = empty_execution_trace(robotContext.dof);
    chunkStartEvents = empty_chunk_start_events();
    queueDrained = false;
    sourceFinishedS = trace.source_finished_replay_t_ms / 1000.0;
    simTimeS = 0.0;

    while simTimeS <= config.maximum_simulation_s
        replayNowMs = simTimeS * 1000.0;

        % Source events are counted only when their recorded ReplaySource due
        % time has arrived.  They are visual evidence of virtual-camera
        % publication; the downstream planner was already recorded in Python.
        while publishedCount < numel(trace.source_events) && ...
                trace.source_events(publishedCount + 1).replay_t_ms <= replayNowMs + 1e-9
            publishedCount = publishedCount + 1;
        end

        % A segment cannot enter the q(t) queue before the fourth/terminal
        % event made it causally available.  This is the key "no future peek"
        % gate in the MATLAB half of the deterministic co-simulation.
        while nextReadySegment <= numel(trace.planned_segments) && ...
                trace.planned_segments(nextReadySegment).time_contract.available_replay_t_ms <= replayNowMs + 1e-9
            segment = trace.planned_segments(nextReadySegment);
            [chunk, ikState] = build_joint_chunk( ...
                segment, trace.source_events, robotContext, ik, ikState, config);
            chunk.ready_replay_t_ms = segment.time_contract.available_replay_t_ms;
            chunk.q_enqueue_execution_t_s = simTimeS;
            queue{end + 1} = chunk; %#ok<AGROW>
            nextReadySegment = nextReadySegment + 1;
        end

        % A controller tick is a sampling instant, not a mandatory pause
        % between chunks.  If this tick lies after a chunk end, advance along
        % the same continuous execution-time axis until the owning chunk is
        % found.  The next chunk starts exactly at the previous chunk end,
        % rather than at the next 100 Hz tick.
        [active, queue, started] = advance_active_queue( ...
            active, queue, simTimeS, config.limit_epsilon);
        chunkStartEvents = [chunkStartEvents; started]; %#ok<AGROW>

        if isempty(active)
            commandQ = ikState.last_commanded_q;
            commandQd = zeros(1, robotContext.dof);
            commandQdd = zeros(1, robotContext.dof);
            desiredXYZ = [NaN, NaN, NaN];
            activeSegmentIndex = NaN;
        else
            localTimeS = min(max(simTimeS - active.execution_start_s, 0.0), ...
                active.execution_duration_s);
            [commandQ, commandQd, commandQdd] = evaluate_joint_quintic( ...
                active.coefficients, localTimeS);
            desiredXYZ = desired_base_xyz(active, localTimeS);
            activeSegmentIndex = active.segment_index;
            ikState.last_commanded_q = commandQ;
        end

        [queuedDurationS, activeRemainingS, startupRemainingS, executionBacklogS] = ...
            queue_metrics(active, queue, simTimeS);
        fkPose = getTransform(robotContext.model, commandQ, robotContext.end_effector);
        execution = append_execution_sample(execution, simTimeS, replayNowMs, ...
            publishedCount, nextReadySegment - 1, numel(queue), queuedDurationS, ...
            activeRemainingS, startupRemainingS, executionBacklogS, active, activeSegmentIndex, ...
            commandQ, commandQd, commandQdd, desiredXYZ, fkPose(1:3,4)');

        % With no successor, retain the terminal chunk long enough to record
        % its exact endpoint.  A successor was already installed above at the
        % mathematically shared boundary, so it never costs an idle tick.
        if ~isempty(active) && simTimeS - active.execution_start_s >= ...
                active.execution_duration_s - config.limit_epsilon && isempty(queue)
            active = [];
        end

        if simTimeS >= sourceFinishedS && ...
                nextReadySegment > numel(trace.planned_segments) && ...
                isempty(queue) && isempty(active)
            queueDrained = true;
            break;
        end
        simTimeS = simTimeS + config.controller_period_s;
    end

    if simTimeS > config.maximum_simulation_s
        error('MonoTeach:ReplayExecutionTimeout', ...
            'Virtual controller did not drain the q(t) queue before maximum_simulation_s.');
    end
    run = struct();
    run.artifact_type = 'RealtimeReplayExecutionRun';
    run.trace_path = char(string(tracePath));
    run.trace = trace;
    run.robot_context = robotContext;
    run.config = config;
    run.execution = execution;
    run.chunk_start_events = chunkStartEvents;
    run.startup = startup;
    run.summary = summarize_execution(execution, trace, config, chunkStartEvents, queueDrained);
end


function trace = load_trace(tracePath)

    try
        trace = jsondecode(fileread(tracePath));
    catch exception
        error('MonoTeach:ReplayTraceReadFailed', 'Cannot read replay trace: %s', exception.message);
    end
    required = {'schema_version','source_events','planned_segments','source_finished_replay_t_ms'};
    if ~isstruct(trace) || ~all(isfield(trace, required)) || ...
            ~strcmp(string(trace.schema_version), 'stage3.4_replay_execution_trace_v1')
        error('MonoTeach:InvalidReplayTrace', 'Trace is not the supported Stage 3.4 replay artifact.');
    end
    if ~isstruct(trace.source_events) || ~isstruct(trace.planned_segments)
        error('MonoTeach:InvalidReplayTrace', 'Trace events and planned segments must be JSON object arrays.');
    end
end


function validate_config(config, context)

    required = {'execution_time_scale','controller_period_s','maximum_simulation_s', ...
        'ik_position_tolerance_m','render_rate_hz','max_velocity_rad_s','max_acceleration_rad_s2','limit_epsilon'};
    if ~isstruct(config) || ~all(isfield(config, required)) || ...
            any(~isfinite([config.execution_time_scale, config.controller_period_s, ...
            config.maximum_simulation_s, config.ik_position_tolerance_m, config.render_rate_hz, config.limit_epsilon])) || ...
            config.execution_time_scale <= 0 || config.controller_period_s <= 0 || ...
            config.maximum_simulation_s <= 0 || config.ik_position_tolerance_m <= 0 || ...
            config.render_rate_hz <= 0 || config.limit_epsilon < 0 || ...
            ~isequal(size(config.max_velocity_rad_s), [1, context.dof]) || ...
            ~isequal(size(config.max_acceleration_rad_s2), [1, context.dof]) || ...
            any(config.max_velocity_rad_s <= 0) || any(config.max_acceleration_rad_s2 <= 0)
        error('MonoTeach:InvalidReplayExecutionConfig', ...
            'Replay execution config needs positive finite timing, IK, and joint limits.');
    end
    if ~isfield(config, 'enable_start_synchronization') || ...
            ~islogical(config.enable_start_synchronization) || ...
            ~isscalar(config.enable_start_synchronization)
        error('MonoTeach:InvalidReplayExecutionConfig', ...
            'enable_start_synchronization must be a scalar logical.');
    end
end


function [ik, state] = make_position_only_ik(context)

    ikConfig = default_ik_config();
    ik = inverseKinematics('RigidBodyTree', context.model);
    try
        ik.SolverAlgorithm = ikConfig.solver_algorithm;
        ik.SolverParameters.AllowRandomRestart = false;
    catch
        % Older MATLAB releases may expose different solver parameters.  The
        % call remains deterministic because each accepted q is independently
        % FK-checked below; this catch does not silently change task weights.
    end
    homeQ = row_configuration(homeConfiguration(context.model));
    homePose = getTransform(context.model, homeQ, context.end_effector);
    state = struct('q_by_source_index', {{}}, 'last_source_index', -1, ...
        'last_success_q', homeQ, 'last_commanded_q', homeQ, ...
        'anchor_q', homeQ, 'target_orientation', homePose(1:3,1:3), ...
        'weights', ikConfig.weights);
end


function [chunk, state] = build_joint_chunk(segment, events, context, ik, state, config)
%BUILD_JOINT_CHUNK Convert one ready Cartesian segment into one executable q(t).
%
% The q endpoints are solved only once and cached by source index.  For an
% interior segment, qd/qdd at both endpoints are estimated from the same
% overlapping triplets used by the Cartesian planner.  Dividing source qd by a
% single global execution-time scale (and qdd by its square) keeps adjacent
% chunks C2-continuous while permitting robot time to differ from human time.

    startIndex = segment.source_start_index + 1; % Python index -> MATLAB index
    endIndex = segment.source_end_index + 1;
    if strcmp(string(segment.start_state_method), 'run_start_zero_state')
        required = [startIndex, endIndex, endIndex + 1];
    else
        required = [startIndex - 1, startIndex, endIndex, endIndex + 1];
    end
    if strcmp(string(segment.finalization_reason), 'terminal_stop')
        required = required(required <= numel(events));
    end
    for index = unique(required, 'sorted')
        [state, ~] = ensure_source_q(index, events, context, ik, state, config);
    end

    qStart = state.q_by_source_index{startIndex};
    qEnd = state.q_by_source_index{endIndex};
    [qdStartSource, qddStartSource] = endpoint_joint_state( ...
        segment.start_state_method, startIndex, events, state.q_by_source_index, context.dof);
    [qdEndSource, qddEndSource] = endpoint_joint_state( ...
        segment.end_state_method, endIndex, events, state.q_by_source_index, context.dof);

    sourceDurationS = (segment.time_contract.source_end_t_ms - ...
        segment.time_contract.source_start_t_ms) / 1000.0;
    executionDurationS = sourceDurationS * config.execution_time_scale;
    qdStart = qdStartSource / config.execution_time_scale;
    qdEnd = qdEndSource / config.execution_time_scale;
    qddStart = qddStartSource / config.execution_time_scale^2;
    qddEnd = qddEndSource / config.execution_time_scale^2;
    coefficients = quintic_coefficients(qStart, qdStart, qddStart, ...
        qEnd, qdEnd, qddEnd, executionDurationS);
    chunk = struct('kind', 'TRACKING', 'segment_index', segment.segment_index, ...
        'source_start_index', segment.source_start_index, ...
        'source_end_index', segment.source_end_index, ...
        'source_start_t_ms', segment.time_contract.source_start_t_ms, ...
        'source_end_t_ms', segment.time_contract.source_end_t_ms, ...
        'profile', segment.profile, 'coefficients', coefficients, ...
        'execution_duration_s', executionDurationS, 'execution_start_s', NaN, ...
        'q_start_rad', qStart, 'q_end_rad', qEnd, ...
        'qd_start_rad_s', qdStart, 'qd_end_rad_s', qdEnd, ...
        'qdd_start_rad_s2', qddStart, 'qdd_end_rad_s2', qddEnd);
    assert_chunk_safety(chunk, context, config);
end


function startup = resolve_startup(trace, config)
%RESOLVE_STARTUP Interpret optional Python PREPARE evidence without guessing.

    startup = struct('enabled', false, 'prepare_duration_s', 0.0, ...
        'tracking_start_replay_s', 0.0, 'prepare_observation_count', 0, ...
        'approach_chunk', []);
    if ~config.enable_start_synchronization || ~isfield(trace, 'startup') || ...
            ~isstruct(trace.startup) || ~isfield(trace.startup, 'enabled') || ...
            ~logical(trace.startup.enabled)
        return;
    end
    required = {'prepare_duration_ms','tracking_start_replay_t_ms', ...
        'start_source_index','prepare_observations'};
    if ~all(isfield(trace.startup, required)) || ...
            ~isscalar(trace.startup.prepare_duration_ms) || ...
            ~isscalar(trace.startup.tracking_start_replay_t_ms) || ...
            ~isfinite(trace.startup.prepare_duration_ms) || ...
            trace.startup.prepare_duration_ms <= 0 || ...
            abs(trace.startup.prepare_duration_ms - trace.startup.tracking_start_replay_t_ms) > 1e-9
        error('MonoTeach:InvalidStartupTrace', ...
            'Startup trace must reserve one positive, consistent PREPARE duration.');
    end
    if trace.startup.start_source_index ~= 0 || isempty(trace.source_events) || ...
            trace.source_events(1).replay_t_ms < trace.startup.prepare_duration_ms - 1e-9
        error('MonoTeach:InvalidStartupTrace', ...
            'Formal source P0 must begin after the derived PREPARE interval.');
    end
    startup.enabled = true;
    startup.prepare_duration_s = trace.startup.prepare_duration_ms / 1000.0;
    startup.tracking_start_replay_s = trace.startup.tracking_start_replay_t_ms / 1000.0;
    startup.prepare_observation_count = numel(trace.startup.prepare_observations);
end


function [chunk, state, startup] = build_home_to_start_chunk(startup, events, context, ik, state, config)
%BUILD_HOME_TO_START_CHUNK Smoothly occupy PREPARE time from home to first pose.
%
% PREPARE observations never reach the rolling planner.  Their only purpose
% here is to reserve enough virtual time for the robot to settle at P0 before
% canonical source replay begins.

    chunk = [];
    if ~startup.enabled
        return;
    end
    [state, qStart] = ensure_source_q(1, events, context, ik, state, config);
    qHome = context.home_q;
    durationS = startup.prepare_duration_s;
    chunk = struct('kind', 'HOME_TO_START', 'segment_index', NaN, ...
        'source_start_index', NaN, 'source_end_index', 0, ...
        'source_start_t_ms', NaN, 'source_end_t_ms', events(1).source_t_ms, ...
        'ready_replay_t_ms', NaN, 'q_enqueue_execution_t_s', 0.0, ...
        'profile', [], 'coefficients', quintic_coefficients( ...
            qHome, zeros(1, context.dof), zeros(1, context.dof), ...
            qStart, zeros(1, context.dof), zeros(1, context.dof), durationS), ...
        'execution_duration_s', durationS, 'execution_start_s', 0.0, ...
        'q_start_rad', qHome, 'q_end_rad', qStart, ...
        'qd_start_rad_s', zeros(1, context.dof), 'qd_end_rad_s', zeros(1, context.dof), ...
        'qdd_start_rad_s2', zeros(1, context.dof), 'qdd_end_rad_s2', zeros(1, context.dof));
    assert_chunk_safety(chunk, context, config);
    startup.approach_chunk = chunk;
end


function [state, q] = ensure_source_q(index, events, context, ik, state, config)

    if index < 1 || index > numel(events)
        error('MonoTeach:RealtimeIKIndex', 'Requested source q lies outside trace events.');
    end
    if numel(state.q_by_source_index) >= index && ~isempty(state.q_by_source_index{index})
        q = state.q_by_source_index{index};
        return;
    end
    event = events(index);
    if ~event.valid || ~event.inside_workspace
        error('MonoTeach:RealtimeIKBarrier', 'A q(t) chunk cannot solve an invalid/outside event.');
    end
    if index ~= state.last_source_index + 1
        % A source-index jump proves a barrier occurred.  Re-anchor rather
        % than use the last pre-barrier IK seed as if the arm had crossed it.
        state.last_success_q = state.anchor_q;
    end
    xyz = workspace_to_base_xyz(event.x_mm, event.y_mm);
    targetPose = eye(4);
    targetPose(1:3,1:3) = state.target_orientation;
    targetPose(1:3,4) = xyz';
    [candidate, info] = ik(context.end_effector, targetPose, state.weights, state.last_success_q);
    q = row_configuration(candidate);
    fk = getTransform(context.model, q, context.end_effector);
    positionError = norm(fk(1:3,4)' - xyz);
    withinLimits = all(q >= context.joint_limits(:,1)' - config.limit_epsilon & ...
        q <= context.joint_limits(:,2)' + config.limit_epsilon);
    if positionError > config.ik_position_tolerance_m || ~withinLimits
        error('MonoTeach:RealtimeIKFailure', ...
            'Source index %d failed Position-Only IK (status %s, FK %.3g m).', ...
            index - 1, string(info.Status), positionError);
    end
    state.q_by_source_index{index} = q;
    state.last_source_index = index;
    state.last_success_q = q;
end


function xyz = workspace_to_base_xyz(xMm, yMm)

    task = default_task_plane_config();
    planeXYM = task.scale * ([xMm, yMm] - task.workspace_center_mm);
    baseH = task.T_base_taskplane * [planeXYM, 0.0, 1.0]';
    xyz = baseH(1:3)';
end


function [qd, qdd] = endpoint_joint_state(method, index, events, qCache, dof)

    if strcmp(string(method), 'run_start_zero_state') || ...
            strcmp(string(method), 'terminal_zero_state')
        qd = zeros(1, dof); qdd = zeros(1, dof); return;
    end
    if ~strcmp(string(method), 'nonuniform_quadratic_three_point_v1')
        error('MonoTeach:UnknownJointBoundaryMethod', 'Unknown source state method %s.', string(method));
    end
    if index <= 1 || index >= numel(events) || isempty(qCache{index-1}) || ...
            isempty(qCache{index}) || isempty(qCache{index+1})
        error('MonoTeach:JointBoundaryLookaheadMissing', ...
            'Centered joint state requires cached predecessor and successor q.');
    end
    h0 = (events(index).source_t_ms - events(index-1).source_t_ms) / 1000.0;
    h1 = (events(index+1).source_t_ms - events(index).source_t_ms) / 1000.0;
    q0 = qCache{index-1}; q1 = qCache{index}; q2 = qCache{index+1};
    span = h0 + h1;
    % This is the vector form of the same non-uniform three-point quadratic
    % estimator used in Python.  It estimates source qdot/qdd directly; it
    % does not recursively require a velocity at the following waypoint.
    qd = -h1*q0/(h0*span) + (h1-h0)*q1/(h0*h1) + h0*q2/(h1*span);
    qdd = 2.0 * (q0/(h0*span) - q1/(h0*h1) + q2/(h1*span));
end


function coefficients = quintic_coefficients(q0, qd0, qdd0, q1, qd1, qdd1, duration)

    c0 = q0; c1 = qd0; c2 = qdd0 / 2.0;
    positionResidual = q1 - (c0 + c1*duration + c2*duration^2);
    velocityResidual = qd1 - (c1 + 2.0*c2*duration);
    accelerationResidual = qdd1 - 2.0*c2;
    c3 = (10.0*positionResidual - 4.0*velocityResidual*duration + ...
        0.5*accelerationResidual*duration^2) / duration^3;
    c4 = (-15.0*positionResidual + 7.0*velocityResidual*duration - ...
        accelerationResidual*duration^2) / duration^4;
    c5 = (6.0*positionResidual - 3.0*velocityResidual*duration + ...
        0.5*accelerationResidual*duration^2) / duration^5;
    coefficients = [c0; c1; c2; c3; c4; c5];
end


function [q, qd, qdd] = evaluate_joint_quintic(coefficients, t)

    q = coefficients(1,:) + coefficients(2,:)*t + coefficients(3,:)*t^2 + ...
        coefficients(4,:)*t^3 + coefficients(5,:)*t^4 + coefficients(6,:)*t^5;
    qd = coefficients(2,:) + 2*coefficients(3,:)*t + 3*coefficients(4,:)*t^2 + ...
        4*coefficients(5,:)*t^3 + 5*coefficients(6,:)*t^4;
    qdd = 2*coefficients(3,:) + 6*coefficients(4,:)*t + ...
        12*coefficients(5,:)*t^2 + 20*coefficients(6,:)*t^3;
end


function xyz = desired_base_xyz(chunk, localTimeS)

    if isempty(chunk.profile)
        % Home-to-Start is intentionally joint-space only.  Do not draw a
        % fictitious Cartesian reference for this separate setup motion.
        xyz = [NaN, NaN, NaN];
        return;
    end

    sourceTimeMs = chunk.source_start_t_ms + ...
        localTimeS / chunk.execution_duration_s * ...
        (chunk.source_end_t_ms - chunk.source_start_t_ms);
    elapsedS = (sourceTimeMs - chunk.profile.start.t_ms) / 1000.0;
    x = polynomial_value(chunk.profile.coefficients_x, elapsedS);
    y = polynomial_value(chunk.profile.coefficients_y, elapsedS);
    xyz = workspace_to_base_xyz(x, y);
end


function value = polynomial_value(coefficients, t)

    value = coefficients(1) + coefficients(2)*t + coefficients(3)*t^2 + ...
        coefficients(4)*t^3 + coefficients(5)*t^4 + coefficients(6)*t^5;
end


function assert_chunk_safety(chunk, context, config)

    sampleTimes = unique([0.0:config.controller_period_s:chunk.execution_duration_s, ...
        chunk.execution_duration_s]);
    q = zeros(numel(sampleTimes), context.dof);
    qd = q; qdd = q;
    for i = 1:numel(sampleTimes)
        [q(i,:), qd(i,:), qdd(i,:)] = evaluate_joint_quintic(chunk.coefficients, sampleTimes(i));
    end
    if any(q < context.joint_limits(:,1)' - config.limit_epsilon, 'all') || ...
            any(q > context.joint_limits(:,2)' + config.limit_epsilon, 'all') || ...
            any(abs(qd) > config.max_velocity_rad_s + config.limit_epsilon, 'all') || ...
            any(abs(qdd) > config.max_acceleration_rad_s2 + config.limit_epsilon, 'all')
        error('MonoTeach:RealtimeJointChunkLimit', ...
            ['q(t) chunk %d violates planning limits. Increase the single ' ...
             'execution_time_scale; do not silently alter source timestamps.'], chunk.segment_index);
    end
end


function [active, queue, started] = advance_active_queue(active, queue, nowS, epsilon)
%ADVANCE_ACTIVE_QUEUE Find the q(t) chunk owning one continuous controller time.
%
% A 100 Hz controller samples the trajectory; it does not insert a 10 ms hold
% after every polynomial.  When a tick crosses one or more chunk boundaries,
% this loop carries the residual time into the successor.  This is what removes
% the old artificial ``number of chunks * controller period`` tail delay.

    started = empty_chunk_start_events();
    if isempty(active) && ~isempty(queue)
        [active, queue, startEvent] = start_next_chunk(queue, nowS);
        started(end+1,1) = startEvent;
    end
    while ~isempty(active)
        activeEndS = active.execution_start_s + active.execution_duration_s;
        if nowS < activeEndS - epsilon || isempty(queue)
            return;
        end
        [active, queue, startEvent] = start_next_chunk(queue, activeEndS);
        started(end+1,1) = startEvent;
    end
end


function [active, queue, startEvent] = start_next_chunk(queue, startS)

    active = queue{1};
    queue(1) = [];
    active.execution_start_s = startS;
    startEvent = chunk_start_event_template();
    startEvent.segment_index = active.segment_index;
    startEvent.source_start_t_ms = active.source_start_t_ms;
    startEvent.source_end_t_ms = active.source_end_t_ms;
    startEvent.ready_replay_t_ms = active.ready_replay_t_ms;
    startEvent.q_enqueue_execution_t_s = active.q_enqueue_execution_t_s;
    startEvent.execution_start_s = startS;
    startEvent.execution_duration_s = active.execution_duration_s;
    startEvent.execution_finish_s = startS + active.execution_duration_s;
end


function [queuedDurationS, activeRemainingS, startupRemainingS, backlogS] = queue_metrics(active, queue, nowS)

    if isempty(queue)
        queuedDurationS = 0.0;
    else
        queuedDurationS = sum(cellfun(@(chunk) chunk.execution_duration_s, queue));
    end
    activeRemainingS = 0.0;
    startupRemainingS = 0.0;
    if ~isempty(active)
        remaining = max(active.execution_duration_s - (nowS - active.execution_start_s), 0.0);
        if strcmp(active.kind, 'HOME_TO_START')
            % Setup is real robot motion, but it is not a delayed q(t) FIFO
            % item.  Reporting it separately keeps queue latency honest.
            startupRemainingS = remaining;
        else
            activeRemainingS = remaining;
        end
    end
    backlogS = queuedDurationS + activeRemainingS;
end


function events = empty_chunk_start_events()

    events = repmat(chunk_start_event_template(), 0, 1);
end


function event = chunk_start_event_template()

    event = struct('segment_index', NaN, 'source_start_t_ms', NaN, ...
        'source_end_t_ms', NaN, 'ready_replay_t_ms', NaN, ...
        'q_enqueue_execution_t_s', NaN, 'execution_start_s', NaN, ...
        'execution_duration_s', NaN, 'execution_finish_s', NaN);
end


function execution = empty_execution_trace(dof)

    execution = struct('t_s', zeros(0,1), 'replay_t_ms', zeros(0,1), ...
        'source_published_count', zeros(0,1), 'segments_ready_count', zeros(0,1), ...
        'queued_chunk_count', zeros(0,1), 'queued_duration_s', zeros(0,1), ...
        'active_remaining_s', zeros(0,1), 'startup_remaining_s', zeros(0,1), ...
        'execution_backlog_s', zeros(0,1), ...
        'active_segment_index', zeros(0,1), 'active_source_start_t_ms', zeros(0,1), ...
        'active_source_end_t_ms', zeros(0,1), 'active_ready_replay_t_ms', zeros(0,1), ...
        'active_execution_start_s', zeros(0,1), 'phase', strings(0,1), ...
        'q_rad', zeros(0,dof), 'qd_rad_s', zeros(0,dof), 'qdd_rad_s2', zeros(0,dof), ...
        'desired_base_xyz_m', zeros(0,3), 'fk_xyz_m', zeros(0,3));
end


function execution = append_execution_sample(execution, t, replayMs, published, ready, queued, ...
        queuedDurationS, activeRemainingS, startupRemainingS, backlogS, active, activeIndex, ...
        q, qd, qdd, desired, fk)

    if isempty(active)
        activeSourceStartMs = NaN; activeSourceEndMs = NaN;
        activeReadyMs = NaN; activeStartS = NaN;
        phase = "HOLD";
    else
        activeSourceStartMs = active.source_start_t_ms;
        activeSourceEndMs = active.source_end_t_ms;
        activeReadyMs = active.ready_replay_t_ms;
        activeStartS = active.execution_start_s;
        phase = string(active.kind);
    end
    execution.t_s(end+1,1) = t;
    execution.replay_t_ms(end+1,1) = replayMs;
    execution.source_published_count(end+1,1) = published;
    execution.segments_ready_count(end+1,1) = ready;
    execution.queued_chunk_count(end+1,1) = queued;
    execution.queued_duration_s(end+1,1) = queuedDurationS;
    execution.active_remaining_s(end+1,1) = activeRemainingS;
    execution.startup_remaining_s(end+1,1) = startupRemainingS;
    execution.execution_backlog_s(end+1,1) = backlogS;
    execution.active_segment_index(end+1,1) = activeIndex;
    execution.active_source_start_t_ms(end+1,1) = activeSourceStartMs;
    execution.active_source_end_t_ms(end+1,1) = activeSourceEndMs;
    execution.active_ready_replay_t_ms(end+1,1) = activeReadyMs;
    execution.active_execution_start_s(end+1,1) = activeStartS;
    execution.phase(end+1,1) = phase;
    execution.q_rad(end+1,:) = q;
    execution.qd_rad_s(end+1,:) = qd;
    execution.qdd_rad_s2(end+1,:) = qdd;
    execution.desired_base_xyz_m(end+1,:) = desired;
    execution.fk_xyz_m(end+1,:) = fk;
end


function summary = summarize_execution(execution, trace, config, chunkStarts, queueDrained)

    validDesired = all(isfinite(execution.desired_base_xyz_m), 2);
    fkDeviation = vecnorm(execution.fk_xyz_m(validDesired,:) - ...
        execution.desired_base_xyz_m(validDesired,:), 2, 2);
    sourceFinishedS = trace.source_finished_replay_t_ms / 1000.0;
    sourceEndIndex = find(execution.t_s <= sourceFinishedS + config.limit_epsilon, 1, 'last');
    if isempty(sourceEndIndex), sourceEndIndex = 1; end
    if isempty(chunkStarts)
        totalScheduledDurationS = 0.0;
        totalInterChunkIdleS = NaN;
        maxReadyHandoffGapS = NaN;
        maxReadyToExecutionStartS = NaN;
    else
        totalScheduledDurationS = sum([chunkStarts.execution_duration_s]);
        % This includes legitimate waits for a later source observation.  It is
        % reported separately from max_ready_handoff_gap_s, which isolates the
        % scheduler defect fixed in this stage.
        totalInterChunkIdleS = execution.t_s(end) - chunkStarts(1).execution_start_s - ...
            totalScheduledDurationS;
        maxReadyToExecutionStartS = max([chunkStarts.execution_start_s] - ...
            [chunkStarts.ready_replay_t_ms] / 1000.0);
        if numel(chunkStarts) < 2
            maxReadyHandoffGapS = 0.0;
        else
            handoffGapsS = [chunkStarts(2:end).execution_start_s] - ...
                [chunkStarts(1:end-1).execution_finish_s];
            successorWasReady = [chunkStarts(2:end).ready_replay_t_ms] / 1000.0 <= ...
                [chunkStarts(1:end-1).execution_finish_s] + config.limit_epsilon;
            if any(successorWasReady)
                maxReadyHandoffGapS = max(handoffGapsS(successorWasReady));
            else
                maxReadyHandoffGapS = 0.0;
            end
        end
    end
    summary = struct('controller_sample_count', numel(execution.t_s), ...
        'controller_period_s', config.controller_period_s, ...
        'source_event_count', numel(trace.source_events), ...
        'planned_segment_count', numel(trace.planned_segments), ...
        'max_queued_chunk_count', max(execution.queued_chunk_count), ...
        'max_execution_backlog_s', max(execution.execution_backlog_s), ...
        'backlog_at_source_finished_s', execution.execution_backlog_s(sourceEndIndex), ...
        'total_scheduled_execution_duration_s', totalScheduledDurationS, ...
        'total_inter_chunk_idle_s', totalInterChunkIdleS, ...
        'max_ready_handoff_gap_s', maxReadyHandoffGapS, ...
        'max_ready_to_execution_start_s', maxReadyToExecutionStartS, ...
        'execution_end_s', execution.t_s(end), ...
        'source_finished_replay_s', sourceFinishedS, ...
        'tail_latency_s', execution.t_s(end) - sourceFinishedS, ...
        'robot_started_before_source_finished', any( ...
            isfinite(execution.active_segment_index) & execution.t_s < sourceFinishedS), ...
        'queue_drained', queueDrained, ...
        'mean_fk_to_cartesian_reference_m', mean(fkDeviation), ...
        'max_fk_to_cartesian_reference_m', max(fkDeviation));
end

