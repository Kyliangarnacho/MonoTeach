function run = run_banter_tcp_executor(fixturePath, port, config)
%RUN_BANTER_TCP_EXECUTOR Single-flight Task 7 Legacy5 digital-twin executor.
%   It never queues or preempts.  Task 6 trajectories are precompiled at
%   startup; each accepted command merely selects a structurally matching
%   cached q(t), then completion is emitted after its final neutral sample.

    if nargin < 1 || isempty(fixturePath), error('MonoTeach:BanterFixtureRequired', 'Pass Task 6 fixture path.'); end
    if nargin < 2 || isempty(port), port = 51012; end
    if nargin < 3 || isempty(config), config = struct(); end
    context = load_robot_context('legacy5');
    library = load_banter_execution_library(fixturePath, context);
    config = execution_config(config);
    server = tcpserver("127.0.0.1", port, "Timeout", 1.0);
    configureTerminator(server, "LF"); cleaner = onCleanup(@() delete_if_valid(server)); %#ok<NASGU>
    fprintf('MonoTeach Banter Task 7 executor listening on 127.0.0.1:%d\n', port);

    visualizer = []; figureHandle = []; renderingEnabled = config.show_figure;
    if renderingEnabled
        try
            figureHandle = figure('Name', 'MonoTeach Banter Task 7 Executor');
            ax = axes(figureHandle); grid(ax, 'on'); view(ax, 3); axis(ax, 'equal');
            visualizer = live_legacy5_visualizer('create', context.model, context.home_q, ax);
        catch exception
            renderingEnabled = false;
            warning('MonoTeach:BanterRenderingDisabled', 'Task 7 renderer unavailable: %s', exception.message);
        end
    end

    session = ''; nextSequence = 0; replySequence = 0; receiveBuffer = ''; active = []; endSeen = false;
    q = context.home_q; renderedAt = -Inf; runSamples = 0; wall = tic;
    while toc(wall) <= config.maximum_live_s
        nowS = toc(wall);
        if server.Connected && server.NumBytesAvailable > 0
            bytes = read(server, server.NumBytesAvailable, "uint8");
            receiveBuffer = [receiveBuffer char(bytes(:)')]; %#ok<AGROW>
        end
        processed = 0;
        while server.Connected && processed < config.max_messages_per_loop
            newline = strfind(receiveBuffer, sprintf('\n')); if isempty(newline), break, end
            line = strtrim(receiveBuffer(1:newline(1)-1)); receiveBuffer = receiveBuffer(newline(1)+1:end);
            if isempty(line), continue, end
            processed = processed + 1;
            try
                message = jsondecode(line);
                [accepted, session, nextSequence, reason] = accept_banter_message(message, session, nextSequence);
                if ~accepted
                    if ~isempty(session), [replySequence] = send_banter_reply(server, session, replySequence, 'PROTOCOL_ERROR', struct('reason', reason)); end
                    continue
                end
                kind = string(message.kind); payload = message.payload;
                if kind == "HELLO"
                    [replySequence] = send_banter_reply(server, session, replySequence, 'CONNECTED', struct('port', port, 'library_entries', numel(library.entries)));
                elseif kind == "EXECUTE_STYLED_MOTION"
                    if ~isempty(active)
                        [replySequence] = send_banter_reply(server, session, replySequence, 'MOTION_BUSY', active.payload);
                    else
                        [entry, matchReason] = find_execution_entry(library, payload);
                        if isempty(entry)
                            [replySequence] = send_banter_reply(server, session, replySequence, 'MOTION_REJECTED', reject_payload(payload, matchReason));
                        else
                            active = struct('payload', command_provenance(payload), 'trace', replay_compiled_banter_motion(entry.compiled), ...
                                'started_wall_s', nowS, 'next_index', 1, 'started_sent', false);
                            [replySequence] = send_banter_reply(server, session, replySequence, 'MOTION_ACCEPTED', active.payload);
                        end
                    end
                elseif kind == "END_SESSION"
                    endSeen = true;
                else
                    [replySequence] = send_banter_reply(server, session, replySequence, 'PROTOCOL_ERROR', struct('reason', ['unsupported kind ' char(kind)]));
                end
            catch exception
                [replySequence] = send_banter_reply(server, session, replySequence, 'PROTOCOL_ERROR', struct('reason', exception.message));
            end
        end

        if ~isempty(active)
            elapsed = nowS - active.started_wall_s;
            if ~active.started_sent
                active.started_sent = true;
                [replySequence] = send_banter_reply(server, session, replySequence, 'MOTION_STARTED', active.payload);
            end
            while active.next_index <= active.trace.sample_count && active.trace.t_s(active.next_index) <= elapsed + config.limit_epsilon
                q = active.trace.q_rad(active.next_index, :);
                active.next_index = active.next_index + 1; runSamples = runSamples + 1;
            end
            if active.next_index > active.trace.sample_count
                q = active.trace.final_q_rad;
                completePayload = active.payload;
                completePayload.actual_duration_s = elapsed;
                completePayload.planned_duration_s = active.trace.duration_s;
                completePayload.executed_sample_count = active.trace.sample_count;
                completePayload.returned_neutral = norm(q - context.home_q, inf) <= 1.0e-8;
                [replySequence] = send_banter_reply(server, session, replySequence, 'MOTION_COMPLETED', completePayload);
                active = [];
            end
        end
        if renderingEnabled && nowS - renderedAt >= 1.0 / config.render_rate_hz
            renderedAt = nowS;
            try
                visualizer = live_legacy5_visualizer('update', visualizer, context.model, q);
                drawnow limitrate;
            catch exception
                renderingEnabled = false;
                [replySequence] = send_banter_reply(server, session, replySequence, 'RENDERING_DISABLED', struct('reason', exception.message));
            end
        end
        if endSeen && isempty(active)
            [replySequence] = send_banter_reply(server, session, replySequence, 'SESSION_FINISHED', struct('returned_neutral', norm(q-context.home_q, inf)<=1.0e-8)); %#ok<NASGU>
            break
        end
        pause(0.001);
    end
    run = struct('queue_drained', isempty(active), 'returned_neutral', norm(q-context.home_q, inf)<=1.0e-8, ...
        'executed_sample_count', runSamples, 'timed_out', toc(wall)>=config.maximum_live_s, 'figure', figureHandle);
end

function config = execution_config(config)
    config.maximum_live_s = field_or(config, 'maximum_live_s', Inf);
    config.max_messages_per_loop = field_or(config, 'max_messages_per_loop', 32);
    config.render_rate_hz = field_or(config, 'render_rate_hz', 25.0);
    config.show_figure = field_or(config, 'show_figure', true);
    config.limit_epsilon = field_or(config, 'limit_epsilon', 1.0e-9);
end
function value = field_or(s, name, fallback), if isfield(s,name), value=s.(name); else, value=fallback; end, end

function [accepted, session, next, reason] = accept_banter_message(message, session, next)
    accepted=false; reason='invalid message';
    required = {'schema_version','session_id','sequence_id','kind','payload'};
    if ~isstruct(message) || ~all(isfield(message, required)) || ~strcmp(string(message.schema_version), 'banter_execution_v1'), return, end
    sequence=double(message.sequence_id); if ~isfinite(sequence)||sequence~=floor(sequence)||sequence<0, reason='invalid sequence_id'; return, end
    incoming=char(string(message.session_id));
    if isempty(session)
        if string(message.kind) ~= "HELLO" || sequence ~= 0, reason='first message must be HELLO sequence 0'; return, end
        session=incoming; next=1;
    elseif ~strcmp(session,incoming) || sequence ~= next
        reason=sprintf('expected session %s sequence %d', session, next); return
    else
        next=next+1;
    end
    accepted=true; reason='';
end

function [next] = send_banter_reply(server, session, sequence, kind, payload)
    next=sequence;
    if ~server.Connected || isempty(session), return, end
    reply=struct('schema_version','banter_execution_v1','session_id',session,'sequence_id',sequence,'kind',kind,'payload',payload);
    writeline(server,jsonencode(reply)); next=sequence+1;
end

function [entry, reason] = find_execution_entry(library, payload)
    entry=[]; reason='MISSING_EXECUTION_FIELDS';
    fields={'behavior_name','variant','macro_name','styled_motion_plan'};
    if ~isstruct(payload) || ~all(isfield(payload, fields)), return, end
    key=char(string(payload.behavior_name)+":"+string(payload.variant));
    index=find(strcmp({library.entries.key}, key),1);
    if isempty(index), reason='UNKNOWN_LIBRARY_KEY'; return, end
    candidate=library.entries(index);
    if ~styled_plan_matches(payload.styled_motion_plan,candidate.source_plan)
        reason='PLAN_SIGNATURE_MISMATCH'; return
    end
    entry=candidate; reason='';
end

function matches = styled_plan_matches(incoming, reference)
    matches=false;
    fields={'behavior_name','variant','macro_name','motion_steps','style'};
    if ~isstruct(incoming)||~all(isfield(incoming,fields)), return, end
    if ~strcmp(string(incoming.behavior_name),string(reference.behavior_name)) || ...
            ~strcmp(string(incoming.variant),string(reference.variant)) || ...
            ~strcmp(string(incoming.macro_name),string(reference.macro_name)), return, end
    a=incoming.motion_steps; b=reference.motion_steps;
    if numel(a)~=numel(b), return, end
    for i=1:numel(a)
        if ~strcmp(string(a(i).kind),string(b(i).kind)) || ~strcmp(string(a(i).target_pose_name),string(b(i).target_pose_name)) || ...
                abs(double(a(i).duration_ms)-double(b(i).duration_ms))>1.0e-9, return, end
    end
    matches=true;
end

function payload = command_provenance(incoming)
    payload=struct(); fields={'execution_id','behavior_id','plan_id','styled_plan_id','behavior_name','variant','macro_name'};
    for i=1:numel(fields)
        if ~isfield(incoming,fields{i}), error('MonoTeach:BanterExecutionProvenance','Missing %s.',fields{i}); end
        payload.(fields{i})=incoming.(fields{i});
    end
end
function payload = reject_payload(incoming, reason)
    try, payload=command_provenance(incoming); catch, payload=struct(); end
    payload.reason=reason;
end
function delete_if_valid(item), if ~isempty(item)&&isvalid(item), delete(item); end, end
