function run = run_live_tcp_executor(port, config)
%RUN_LIVE_TCP_EXECUTOR Python Cartesian stream -> MATLAB IK -> q(t) FIFO.
% Soft-real-time digital twin: bounded TCP dispatch, exact 100 Hz q(t)
% timestamps with catch-up, and low-rate persistent visualization.

    if nargin < 1 || isempty(port), port = 51011; end
    context = load_robot_context('legacy5');
    if nargin < 2 || isempty(config), config = default_replay_execution_config(context); end
    config = live_config(config, context);
    [ik, state] = make_live_ik_state(context);
    server = tcpserver("127.0.0.1", port, "Timeout", 1.0);
    configureTerminator(server, "LF"); cleaner = onCleanup(@() delete_if_valid(server)); %#ok<NASGU>
    fprintf('MonoTeach live MATLAB executor listening on 127.0.0.1:%d\n', port);

    queue = cell(1,0); active = []; q = context.home_q; qd = zeros(1,context.dof); qdd = qd;
    startReadySent=false; endSeen=false; finishedSent=false; sourceCount=0; heartbeatCount=0; receiveBuffer='';
    sourceXY=zeros(0,2); planXY=zeros(0,2); qHistory=zeros(0,context.dof); timeHistory=zeros(0,1); backlogHistory=zeros(0,1); latenessHistory=zeros(0,1);
    executedFK=zeros(0,3); figureHandle=make_live_figure(context,q,state.task_plane); wall=tic; nextTick=0.0; nextRender=0.0; nextFKSample=0.0; renderingEnabled=true;

    while toc(wall) <= config.maximum_live_s
        nowS=toc(wall);
        if server.Connected && server.NumBytesAvailable>0
            bytes=read(server,server.NumBytesAvailable,"uint8"); receiveBuffer=[receiveBuffer char(bytes(:)')]; %#ok<AGROW>
        end
        % TCP byte reads do not imply JSON-message boundaries.  Consume only a
        % bounded number of complete NDJSON facts before returning to control.
        processed=0;
        while server.Connected && processed<config.max_messages_per_loop
            newlineIndex=strfind(receiveBuffer,sprintf('\n')); if isempty(newlineIndex),break,end
            line=strtrim(receiveBuffer(1:newlineIndex(1)-1)); receiveBuffer=receiveBuffer(newlineIndex(1)+1:end); if isempty(line),continue,end
            message=jsondecode(line); [accepted,state,reason]=accept_message(message,state);
            if ~accepted, send_reply(server,state.session_id,"PROTOCOL_ERROR",struct('reason',reason)); continue,end
            processed=processed+1; kind=string(message.kind); payload=message.payload; state.last_message_kind=char(kind); state.last_message_sequence=double(message.sequence_id);
            if mod(double(message.sequence_id)+1,config.ack_every)==0, send_reply(server,state.session_id,"ACK",struct('highest_sequence_id',message.sequence_id)); end
            try
                if kind=="HELLO"
                    send_reply(server,state.session_id,"CONNECTED",struct('port',port));
                elseif kind=="WORKSPACE_GEOMETRY"
                    state.task_plane=live_task_plane_from_workspace(payload,state.task_plane); update_workspace_limits(figureHandle,state.task_plane);
                elseif kind=="START_TARGET"
                    [state,qStart]=solve_live_target(double(payload.x_mm),double(payload.y_mm),payload_z(payload,0.0),state,context,ik,config);
                    active=make_safe_home_chunk(context.home_q,qStart,context,config); active.execution_start_s=nowS; startReadySent=false;
                elseif kind=="SOURCE_HEARTBEAT"
                    heartbeatCount=heartbeatCount+1; state.last_heartbeat=payload;
                elseif kind=="SOURCE_EVENT"
                    state=store_live_event(state,payload); sourceCount=sourceCount+1;
                    state.last_source_capture_t_ms=double(payload.source_t_ms); state.last_source_latency_ms=double(payload.replay_t_ms)-double(payload.source_t_ms);
                    if logical(payload.valid)&&logical(payload.inside_workspace)
                        sourceXY(end+1,:)=[double(payload.x_mm),double(payload.y_mm)]; %#ok<AGROW>
                        % IK is deterministic but not free.  Warm this source
                        % waypoint when its camera fact arrives, rather than
                        % making the later segment admission solve four cached
                        % support points in one burst.
                        if strcmp(state.admission,'OPEN'), [state,~]=ensure_live_q(double(payload.source_index)+1,state,context,ik,config); end
                    end
                elseif kind=="REACQUIRE_TARGET"
                    if ~state.pending_reacquire
                        error('MonoTeach:LiveReacquireState','REACQUIRE_TARGET arrived without a preceding vision boundary.');
                    end
                    state.pending_reacquire_target=payload;
                    send_reply(server,state.session_id,"REACQUIRE_QUEUED",struct('admission',admission_text_live(state)));
                elseif kind=="PLANNED_SEGMENT"
                    if strcmp(state.admission,'OPEN')
                        [chunk,state]=build_live_chunk(payload,state,context,ik,config); chunk.enqueue_wall_s=nowS; queue{end+1}=chunk; %#ok<AGROW>
                        planXY=[planXY;sample_profile_live(payload.profile)]; %#ok<AGROW>
                    end
                elseif kind=="BARRIER"
                    if string(payload.kind)=="PEN_SEMANTIC_TRANSITION"
                        qHold=queue_endpoint_live(active,queue,q);
                        [transitionChunks,state]=build_pen_transition(payload,state,context,ik,config,qHold);
                        transitionChunks=mark_completion_live(transitionChunks,'PEN_TRANSITION_READY');
                        queue=[queue transitionChunks]; %#ok<AGROW>
                    elseif is_recoverable_vision_barrier(payload)
                        % A perception gap remains a spatial boundary.  It is
                        % not, by itself, a reason to discard the whole live
                        % session or the chunks already safety-checked.
                        state.pending_reacquire=true; state.last_vision_reason=char(payload.reason); state.last_vision_source_index=payload_source_index(payload);
                        sourceXY=[sourceXY;NaN NaN]; planXY=[planXY;NaN NaN]; %#ok<AGROW>
                        send_reply(server,state.session_id,"VISION_BOUNDARY",struct('reason',payload.reason,'source_index',payload_source_index(payload),'admission',admission_text_live(state)));
                    else
                        % Input/planning loss is not a simulated actuator ESTOP:
                        % execute only chunks already safety-checked, then hold.
                        state.admission='DRAIN_TO_HOLD'; state.block_reason=char(payload.reason); state.block_message_kind=char(kind); state.block_source_index=payload_source_index(payload);
                        send_reply(server,state.session_id,"DRAIN_TO_HOLD",block_payload_live(state));
                    end
                elseif kind=="END_OF_STREAM"
                    endSeen=true;
                end
            catch exception
                state.admission='DRAIN_TO_HOLD'; state.block_reason=exception.message; state.block_message_kind=char(kind); state.block_source_index=payload_source_index(payload);
                send_reply(server,state.session_id,"PLANNING_BLOCKED",block_payload_live(state));
            end
        end

        % A chunk stores coefficients, not 100 Hz samples.  At each scheduled
        % tick we evaluate its local q(t).  Catch-up prevents drawing/IK work
        % from silently deleting intermediate controller samples.
        while nextTick <= nowS + config.limit_epsilon
            tickS=nextTick;
            % A vision-reacquire target is intentionally held outside the
            % normal PLANNED_SEGMENT path.  Already accepted motion drains
            % first; only then is one safe bridge appended, so new camera
            % points can never build an obsolete tracking backlog behind it.
            if startReadySent && isempty(active) && isempty(queue) && state.pending_reacquire && ~isempty(state.pending_reacquire_target)
                [reacquireChunks,state]=build_reacquire_target_chunks(state.pending_reacquire_target,q,state,context,ik,config);
                queue=mark_completion_live(reacquireChunks,'REACQUIRE_READY');
                state.pending_reacquire_target=[];
            end
            if ~isempty(active) && strcmp(active.kind,'HOME_TO_START') && tickS>=active.execution_start_s+active.execution_duration_s-config.limit_epsilon
                [q,qd,qdd]=evaluate_live_quintic(active.coefficients,active.execution_duration_s); active=[];
                if ~startReadySent, startReadySent=true; send_reply(server,state.session_id,"START_READY",struct()); end
            end
            if startReadySent, [active,queue]=advance_tracking_queue(active,queue,tickS,config.limit_epsilon); end
            if ~isempty(active) && tickS>=active.execution_start_s-config.limit_epsilon
                localS=min(max(tickS-active.execution_start_s,0.0),active.execution_duration_s); [q,qd,qdd]=evaluate_live_quintic(active.coefficients,localS);
            else
                qd=zeros(1,context.dof);qdd=qd;
            end
            qHistory(end+1,:)=q; timeHistory(end+1,1)=tickS; backlogHistory(end+1,1)=queue_backlog_live(active,queue,tickS); latenessHistory(end+1,1)=max(0,nowS-tickS); %#ok<AGROW>
            if ~isempty(active) && tickS>=active.execution_start_s+active.execution_duration_s-config.limit_epsilon && isempty(queue)
                completion=completion_kind_live(active); active=[];
                if ~isempty(completion)
                    if strcmp(completion,'REACQUIRE_READY'), state.pending_reacquire=false; end
                    send_reply(server,state.session_id,string(completion),struct('admission',admission_text_live(state)));
                end
            end
            nextTick=nextTick+config.controller_period_s;
        end
        % FK is sampled at display rate, never at every controller tick.  It
        % gives the user an executed end-effector trail without making drawing
        % or repeated FK part of the 100 Hz timing claim.
        if nowS >= nextFKSample
            nextFKSample=nowS+1/config.fk_trail_rate_hz;
            pose=getTransform(context.model,q,context.end_effector); executedFK(end+1,:)=pose(1:3,4)'; %#ok<AGROW>
        end
        if renderingEnabled && nowS>=nextRender
            nextRender=nowS+1/config.render_rate_hz;
            try
                update_live_figure(figureHandle,context,q,sourceXY,planXY,executedFK,timeHistory,qHistory,backlogHistory,sourceCount,numel(queue),active,state,heartbeatCount,latenessHistory);
            catch exception
                % A plot failure must never look like an IK/network failure.
                % Keep q(t) execution alive and tell Python what disappeared.
                renderingEnabled=false;
                send_reply(server,state.session_id,"RENDERING_DISABLED",struct('reason',exception.message));
                warning('MonoTeach:LiveRenderingDisabled','Live rendering disabled: %s',exception.message);
            end
        end
        if endSeen && isempty(active) && isempty(queue) && startReadySent
            if ~finishedSent,send_reply(server,state.session_id,"FINISHED",struct('source_event_count',sourceCount,'heartbeat_count',heartbeatCount));finishedSent=true;end
            break
        end
        pause(0.001);
    end
    timedOut=~finishedSent && isfinite(config.maximum_live_s) && toc(wall)>=config.maximum_live_s;
    if timedOut
        send_reply(server,state.session_id,"EXECUTOR_TIMEOUT",struct('reason','maximum_live_s elapsed before END_OF_STREAM drained','maximum_live_s',config.maximum_live_s,'queued_chunks',numel(queue),'active_kind',active_text_live(active),'admission',state.admission));
    end
    run=struct('q_rad',qHistory,'t_s',timeHistory,'backlog_s',backlogHistory,'scheduler_lateness_s',latenessHistory,'source_event_count',sourceCount,'heartbeat_count',heartbeatCount,'queue_drained',endSeen&&isempty(active)&&isempty(queue),'admission',state.admission,'block_reason',state.block_reason,'timed_out',timedOut,'figure',figureHandle);
end

function config=live_config(config,context)
    % The operator ends a live session with Q.  A finite timeout is available
    % for automated tests, but it must be opt-in and it reports its own cause
    % instead of silently closing Python's TCP socket at 180 seconds.
    config.minimum_start_chunks=1; config.maximum_live_s=value_or(config,'maximum_live_s',Inf); config.render_rate_hz=value_or(config,'render_rate_hz',15.0); config.fk_trail_rate_hz=value_or(config,'fk_trail_rate_hz',15.0); config.max_messages_per_loop=value_or(config,'max_messages_per_loop',32); config.ack_every=value_or(config,'ack_every',8);
    config.home_min_duration_s=value_or(config,'home_min_duration_s',3.0); config.home_safety_margin=value_or(config,'home_safety_margin',1.15); config.pen_lift_duration_s=value_or(config,'pen_lift_duration_s',0.35); config.pen_transfer_min_duration_s=value_or(config,'pen_transfer_min_duration_s',0.20); config.reacquire_min_duration_s=value_or(config,'reacquire_min_duration_s',0.35); config.tracking_safety_margin=value_or(config,'tracking_safety_margin',1.05); config.maximum_tracking_duration_scale=value_or(config,'maximum_tracking_duration_scale',16.0); config.lifted_z_task_mm=value_or(config,'lifted_z_task_mm',12.0);
    if ~isfield(config,'max_velocity_rad_s'),config.max_velocity_rad_s=context.velocity_limits;end
    if ~isfield(config,'max_acceleration_rad_s2'),config.max_acceleration_rad_s2=context.acceleration_limits;end
    if ~isfield(config,'limit_epsilon'),config.limit_epsilon=1e-9;end
end
function v=value_or(s,name,d),if isfield(s,name),v=s.(name);else,v=d;end,end

function [accepted,state,reason]=accept_message(message,state)
    accepted=false;reason='invalid message';
    if ~isstruct(message)||~isfield(message,'schema_version')||~strcmp(string(message.schema_version),'monoteach_live_v1')||~isfield(message,'kind')||~isfield(message,'payload')||~isfield(message,'session_id')||~isfield(message,'sequence_id'),return,end
    sequence=double(message.sequence_id); if ~isfinite(sequence)||sequence~=floor(sequence)||sequence<0,reason='sequence_id must be a non-negative integer';return,end
    session=char(string(message.session_id));
    if isempty(state.session_id)
        if string(message.kind)~="HELLO"||sequence~=0,reason='first message must be HELLO sequence 0';return,end
        state.session_id=session;state.next_sequence_id=1;
    elseif ~strcmp(state.session_id,session)||sequence~=state.next_sequence_id
        reason=sprintf('expected session %s sequence %d',state.session_id,state.next_sequence_id);return
    else
        state.next_sequence_id=state.next_sequence_id+1;
    end
    accepted=true;reason='';
end

function [ik,state]=make_live_ik_state(context)
    ik=inverseKinematics('RigidBodyTree',context.model);try,ik.SolverAlgorithm='LevenbergMarquardt';ik.SolverParameters.AllowRandomRestart=false;catch,end
    homePose=getTransform(context.model,context.home_q,context.end_effector);
    state=struct('events',{{}},'q_cache',{{}},'last_success_q',context.home_q,'last_source_index',-1,'orientation',homePose(1:3,1:3),'weights',[0 0 0 1 1 1],'task_plane',default_task_plane_config(),'session_id','','next_sequence_id',0,'admission','OPEN','block_reason','','block_message_kind','','block_source_index',NaN,'pending_reacquire',false,'pending_reacquire_target',[],'last_vision_reason','','last_vision_source_index',NaN,'last_message_kind','','last_message_sequence',NaN,'last_source_capture_t_ms',NaN,'last_source_latency_ms',NaN,'last_heartbeat',struct());
end
function state=store_live_event(state,payload),index=double(payload.source_index)+1;if numel(state.events)<index,state.events{index}=[];end,state.events{index}=payload;end

function [state,q]=solve_live_target(x,y,zTaskMm,state,context,ik,config)
    xyz=workspace_to_base_live(x,y,zTaskMm,state.task_plane);pose=eye(4);pose(1:3,1:3)=state.orientation;pose(1:3,4)=xyz';seeds={state.last_success_q,context.home_q};bestError=inf;bestStatus="unknown";
    for i=1:numel(seeds)
        [candidate,info]=ik(context.end_effector,pose,state.weights,seeds{i});candidate=row_configuration(candidate);fk=getTransform(context.model,candidate,context.end_effector);err=norm(fk(1:3,4)'-xyz);within=all(candidate>=context.joint_limits(:,1)'-config.limit_epsilon&candidate<=context.joint_limits(:,2)'+config.limit_epsilon);
        if err<bestError,bestError=err;bestStatus=string(info.Status);end
        if err<=config.ik_position_tolerance_m&&within,q=candidate;state.last_success_q=q;return,end
    end
    error('MonoTeach:LiveIK','Position-Only IK failed after deterministic seeds (status %s, best FK %.3g m).',bestStatus,bestError);
end

function [chunk,state]=build_live_chunk(segment,state,context,ik,config)
    a=double(segment.source_start_index)+1;b=double(segment.source_end_index)+1;required=unique([a,b,max(1,a-1),b+1]);
    for i=required,if i<=numel(state.events)&&~isempty(state.events{i})&&logical(state.events{i}.valid)&&logical(state.events{i}.inside_workspace),[state,~]=ensure_live_q(i,state,context,ik,config);end,end
    [state,q0]=ensure_live_q(a,state,context,ik,config);[state,q1]=ensure_live_q(b,state,context,ik,config);duration=(double(segment.time_contract.source_end_t_ms)-double(segment.time_contract.source_start_t_ms))/1000*config.execution_time_scale;
    [qd0,qdd0]=live_joint_state(a,state,context.dof);[qd1,qdd1]=live_joint_state(b,state,context.dof);qd0=qd0/config.execution_time_scale;qd1=qd1/config.execution_time_scale;qdd0=qdd0/config.execution_time_scale^2;qdd1=qdd1/config.execution_time_scale^2;
    % Source timing supplies the first execution-duration proposal, not an
    % unsafe deadline.  Unlike a position/unreachability fault, a qd/qdd-only
    % violation can be repaired honestly by giving the same quintic more time.
    chunk=make_retimed_live_chunk('TRACKING',double(segment.segment_index),q0,qd0,qdd0,q1,qd1,qdd1,duration,context,config);
end
function [state,q]=ensure_live_q(index,state,context,ik,config)
    if numel(state.q_cache)>=index&&~isempty(state.q_cache{index}),q=state.q_cache{index};return,end
    if index>numel(state.events)||isempty(state.events{index}),error('MonoTeach:LiveMissingSource','Required source support not received.');end
    event=state.events{index};if ~logical(event.valid)||~logical(event.inside_workspace),error('MonoTeach:LiveBarrier','Cannot IK a barrier.');end
    if index~=state.last_source_index+1,state.last_success_q=context.home_q;end
    [state,q]=solve_live_target(double(event.x_mm),double(event.y_mm),payload_z(event,pen_z_default(event.pen_state)),state,context,ik,config);state.q_cache{index}=q;state.last_source_index=index;
end
function [qd,qdd]=live_joint_state(index,state,dof)
    qd=zeros(1,dof);qdd=zeros(1,dof);if index<=1||index>=numel(state.events)||index>=numel(state.q_cache)||isempty(state.q_cache{index-1})||isempty(state.q_cache{index})||isempty(state.q_cache{index+1}),return,end
    e0=state.events{index-1};e1=state.events{index};e2=state.events{index+1};if isempty(e0)||isempty(e1)||isempty(e2)||~logical(e0.valid)||~logical(e1.valid)||~logical(e2.valid)||~strcmp(string(e0.pen_state),string(e1.pen_state))||~strcmp(string(e1.pen_state),string(e2.pen_state)),return,end
    h0=(double(e1.source_t_ms)-double(e0.source_t_ms))/1000;h1=(double(e2.source_t_ms)-double(e1.source_t_ms))/1000;if h0<=0||h1<=0,return,end;span=h0+h1;q0=state.q_cache{index-1};q1=state.q_cache{index};q2=state.q_cache{index+1};qd=-h1*q0/(h0*span)+(h1-h0)*q1/(h0*h1)+h0*q2/(h1*span);qdd=2*(q0/(h0*span)-q1/(h0*h1)+q2/(h1*span));
end

function chunk=make_safe_home_chunk(q0,q1,context,config)
    delta=abs(q1-q0);velocityBound=max(1.875*delta./config.max_velocity_rad_s);accelerationBound=max(sqrt(5.7736*delta./config.max_acceleration_rad_s2));duration=max([config.home_min_duration_s,velocityBound,accelerationBound])*config.home_safety_margin;
    chunk=make_retimed_live_chunk('HOME_TO_START',-1,q0,zeros(1,context.dof),zeros(1,context.dof),q1,zeros(1,context.dof),zeros(1,context.dof),duration,context,config);
end
function chunk=make_live_chunk(kind,index,q0,qd0,qdd0,q1,qd1,qdd1,duration),chunk=struct('kind',char(kind),'segment_index',index,'coefficients',live_quintic(q0,qd0,qdd0,q1,qd1,qdd1,duration),'execution_duration_s',duration,'execution_start_s',NaN);end

function [chunks,state]=build_pen_transition(payload,state,context,ik,config,qHold)
    chunks=cell(1,0);newIndex=double(payload.source_index)+1;if newIndex>numel(state.events)||isempty(state.events{newIndex}),return,end;oldIndex=newIndex-1;while oldIndex>=1&&(isempty(state.events{oldIndex})||~logical(state.events{oldIndex}.valid)||~logical(state.events{oldIndex}.inside_workspace)),oldIndex=oldIndex-1;end;if oldIndex<1,return,end
    oldEvent=state.events{oldIndex};newEvent=state.events{newIndex};if strcmp(string(oldEvent.pen_state),string(newEvent.pen_state)),return,end
    [state,~]=ensure_live_q(oldIndex,state,context,ik,config);[state,qNew]=ensure_live_q(newIndex,state,context,ik,config);qOld=qHold;oldZ=payload_z(oldEvent,pen_z_default(oldEvent.pen_state));newZ=payload_z(newEvent,pen_z_default(newEvent.pen_state));
    if strcmp(string(oldEvent.pen_state),"DOWN")
        [state,qLift]=solve_live_target(double(oldEvent.x_mm),double(oldEvent.y_mm),newZ,state,context,ik,config);chunks{end+1}=safe_transition_chunk('PEN_LIFT',qOld,qLift,config.pen_lift_duration_s,context,config);chunks{end+1}=safe_transition_chunk('PEN_TRANSFER',qLift,qNew,config.pen_transfer_min_duration_s,context,config);
    else
        [state,qTransfer]=solve_live_target(double(newEvent.x_mm),double(newEvent.y_mm),oldZ,state,context,ik,config);chunks{end+1}=safe_transition_chunk('PEN_TRANSFER',qOld,qTransfer,config.pen_transfer_min_duration_s,context,config);chunks{end+1}=safe_transition_chunk('PEN_LOWER',qTransfer,qNew,config.pen_lift_duration_s,context,config);
    end
end

function [chunks,state]=build_reacquire_target_chunks(target,currentQ,state,context,ik,config)
%BUILD_REACQUIRE_TARGET_CHUNKS Construct motion only after old FIFO drained.
% The input target comes from fresh post-gap evidence.  It is never treated as
% the next point of the pre-gap Cartesian curve.
    pen=char(string(target.pen_state)); z=payload_z(target,pen_z_default(pen));
    [state,qTarget]=solve_live_target(double(target.x_mm),double(target.y_mm),z,state,context,ik,config);
    oldIndex=previous_valid_event_index(state,numel(state.events));
    chunks=cell(1,0);
    if strcmp(pen,'DOWN') && oldIndex>=1
        oldEvent=state.events{oldIndex}; liftZ=config.lifted_z_task_mm;
        [state,qLiftOld]=solve_live_target(double(oldEvent.x_mm),double(oldEvent.y_mm),liftZ,state,context,ik,config);
        [state,qLiftNew]=solve_live_target(double(target.x_mm),double(target.y_mm),liftZ,state,context,ik,config);
        chunks={safe_transition_chunk('REACQUIRE_LIFT',currentQ,qLiftOld,config.pen_lift_duration_s,context,config), ...
                safe_transition_chunk('REACQUIRE_TRANSFER',qLiftOld,qLiftNew,config.reacquire_min_duration_s,context,config), ...
                safe_transition_chunk('REACQUIRE_LOWER',qLiftNew,qTarget,config.pen_lift_duration_s,context,config)};
    else
        chunks={safe_transition_chunk('REACQUIRE_TRANSFER',currentQ,qTarget,config.reacquire_min_duration_s,context,config)};
    end
end

function index=previous_valid_event_index(state,index)
    while index>=1
        if index<=numel(state.events)&&~isempty(state.events{index})&&logical(state.events{index}.valid)&&logical(state.events{index}.inside_workspace),return,end
        index=index-1;
    end
end

function q=queue_endpoint_live(active,queue,currentQ)
    if ~isempty(queue)
        tail=queue{end}; [q,~,~]=evaluate_live_quintic(tail.coefficients,tail.execution_duration_s); return
    end
    if ~isempty(active)
        [q,~,~]=evaluate_live_quintic(active.coefficients,active.execution_duration_s); return
    end
    q=currentQ;
end

function chunk=safe_transition_chunk(kind,q0,q1,minimumDuration,context,config)
    chunk=make_retimed_live_chunk(kind,NaN,q0,zeros(1,context.dof),zeros(1,context.dof),q1,zeros(1,context.dof),zeros(1,context.dof),minimumDuration,context,config);
end

function chunk=make_retimed_live_chunk(kind,index,q0,qd0,qdd0,q1,qd1,qdd1,requestedDuration,context,config)
    if ~isfinite(requestedDuration)||requestedDuration<=0,error('MonoTeach:LiveDuration','Requested q(t) duration must be positive.');end
    duration=requestedDuration; maximumDuration=requestedDuration*config.maximum_tracking_duration_scale;
    while true
        % Uniformly stretching a quintic by s changes its boundary rates:
        % qd -> qd/s and qdd -> qdd/s^2.  Extending duration alone leaves an
        % over-limit endpoint velocity untouched, so it can never repair the
        % very violation this loop is intended to handle.
        scale=duration/requestedDuration;
        chunk=make_live_chunk(kind,index,q0,qd0/scale,qdd0/scale^2,q1,qd1/scale,qdd1/scale^2,duration);
        violation=live_chunk_limit_violation(chunk,context,config);
        if isempty(violation), return, end
        if strcmp(violation,'position')
            error('MonoTeach:LiveJointPositionLimit','q(t) chunk %g violates a joint-position limit; duration cannot repair reachability.',index);
        end
        if duration>=maximumDuration
            error('MonoTeach:LiveJointRateLimit','q(t) chunk %g still violates %s limits after %.1fx retiming.',index,violation,duration/requestedDuration);
        end
        duration=min(maximumDuration,duration*1.25*config.tracking_safety_margin);
    end
end

function violation=live_chunk_limit_violation(chunk,context,config)
    violation=''; times=unique([0:config.controller_period_s:chunk.execution_duration_s,chunk.execution_duration_s]);
    for i=1:numel(times)
        [q,qd,qdd]=evaluate_live_quintic(chunk.coefficients,times(i));
        if any(q<context.joint_limits(:,1)'-config.limit_epsilon)||any(q>context.joint_limits(:,2)'+config.limit_epsilon),violation='position';return,end
        if any(abs(qd)>config.max_velocity_rad_s+config.limit_epsilon),violation='velocity';return,end
        if any(abs(qdd)>config.max_acceleration_rad_s2+config.limit_epsilon),violation='acceleration';return,end
    end
end
function assert_live_chunk_safety(chunk,context,config),violation=live_chunk_limit_violation(chunk,context,config);if ~isempty(violation),error('MonoTeach:LiveJointChunkLimit','q(t) chunk %g violates %s planning limits.',chunk.segment_index,violation);end,end
function chunks=mark_completion_live(chunks,kind),if isempty(chunks),return,end;chunks{end}.completion_kind=char(kind);end
function kind=completion_kind_live(chunk),kind='';if isfield(chunk,'completion_kind'),kind=char(chunk.completion_kind);end,end
function [active,queue]=advance_tracking_queue(active,queue,tickS,epsilon),if isempty(active)&&~isempty(queue),active=queue{1};queue(1)=[];active.execution_start_s=tickS;end;while ~isempty(active)&&tickS>=active.execution_start_s+active.execution_duration_s-epsilon&&~isempty(queue),finishS=active.execution_start_s+active.execution_duration_s;active=queue{1};queue(1)=[];active.execution_start_s=finishS;end,end

function task=live_task_plane_from_workspace(payload,task),width=double(payload.width_mm);height=double(payload.height_mm);if ~isfinite(width)||~isfinite(height)||width<=0||height<=0,error('MonoTeach:LiveWorkspaceGeometry','Live workspace width/height must be finite and positive.');end;task.workspace_width_mm=width;task.workspace_height_mm=height;task.workspace_center_mm=[width,height]/2;end
function xyz=workspace_to_base_live(x,y,z,task),p=task.scale*([x,y]-task.workspace_center_mm);h=task.T_base_taskplane*[p,z*task.scale,1]';xyz=h(1:3)';end
function xyz=workspace_trace_to_base_live(xy,task,maximumPoints)
    % This is the planned Cartesian reference in robot-base coordinates.  It
    % is intentionally a display artifact: the controller still evaluates
    % q(t), and the green trail below is FK of that executed q(t).
    if isempty(xy),xyz=zeros(0,3);return,end
    if size(xy,1)>maximumPoints
        indices=round(linspace(1,size(xy,1),maximumPoints));xy=xy(indices,:);
    end
    centered=task.scale*(xy-task.workspace_center_mm);
    homogeneous=[centered,zeros(size(centered,1),1),ones(size(centered,1),1)]';
    transformed=task.T_base_taskplane*homogeneous;xyz=transformed(1:3,:)';
end
function z=payload_z(payload,defaultZ),if isfield(payload,'z_task_mm')&&isfinite(double(payload.z_task_mm)),z=double(payload.z_task_mm);else,z=defaultZ;end,end
function z=pen_z_default(pen),if strcmp(string(pen),"UP"),z=12.0;else,z=0.0;end,end
function c=live_quintic(q0,qd0,qdd0,q1,qd1,qdd1,T),c0=q0;c1=qd0;c2=qdd0/2;r=q1-(c0+c1*T+c2*T^2);v=qd1-(c1+2*c2*T);a=qdd1-2*c2;c3=(10*r-4*v*T+.5*a*T^2)/T^3;c4=(-15*r+7*v*T-a*T^2)/T^4;c5=(6*r-3*v*T+.5*a*T^2)/T^5;c=[c0;c1;c2;c3;c4;c5];end
function [q,qd,qdd]=evaluate_live_quintic(c,t),q=c(1,:)+c(2,:)*t+c(3,:)*t^2+c(4,:)*t^3+c(5,:)*t^4+c(6,:)*t^5;qd=c(2,:)+2*c(3,:)*t+3*c(4,:)*t^2+4*c(5,:)*t^3+5*c(6,:)*t^4;qdd=2*c(3,:)+6*c(4,:)*t+12*c(5,:)*t^2+20*c(6,:)*t^3;end
function b=queue_backlog_live(active,queue,now),b=sum(cellfun(@(x)x.execution_duration_s,queue));if ~isempty(active),b=b+max(0,active.execution_duration_s-(now-active.execution_start_s));end,end
function xy=sample_profile_live(p),t=(double(p.end.t_ms)-double(p.start.t_ms))/1000;u=linspace(0,t,max(2,ceil(t*30)));x=evaluate_cartesian_live(p.coefficients_x,u);y=evaluate_cartesian_live(p.coefficients_y,u);xy=[x(:),y(:)];end
function value=evaluate_cartesian_live(c,t),c=double(c(:));value=c(1)+c(2)*t+c(3)*t.^2+c(4)*t.^3+c(5)*t.^4+c(6)*t.^5;end
function send_reply(server,session,kind,payload),if ~server.Connected||isempty(session),return,end;reply=struct('schema_version','monoteach_live_v1','session_id',session,'kind',kind,'payload',payload);writeline(server,jsonencode(reply));end
function index=payload_source_index(payload)
    % Source events carry source_index; planned segments carry their source
    % interval instead.  Prefer the end index because it identifies the point
    % whose q(t) boundary caused the segment admission attempt.
    if isfield(payload,'source_index'),index=double(payload.source_index);
    elseif isfield(payload,'source_end_index'),index=double(payload.source_end_index);
    elseif isfield(payload,'source_start_index'),index=double(payload.source_start_index);
    else,index=NaN;
    end
end
function payload=block_payload_live(state),payload=struct('reason',state.block_reason,'message_kind',state.block_message_kind,'source_index',state.block_source_index,'admission',state.admission);end
function yes=is_recoverable_vision_barrier(payload),yes=isfield(payload,'kind')&&(string(payload.kind)=="OBSERVATION_INVALID"||string(payload.kind)=="OUTSIDE_WORKSPACE");end
function text=admission_text_live(state),if strcmp(state.admission,'OPEN')&&state.pending_reacquire,text='OPEN_REACQUIRE_PENDING';else,text=state.admission;end,end

function f=make_live_figure(context,q,task)
    f=figure('Name','MonoTeach Live Python -> MATLAB q(t)','Color','w','Position',[60,60,1500,920]);tiledlayout(2,2,'TileSpacing','compact');ax1=nexttile;hold(ax1,'on');grid(ax1,'on');axis(ax1,'equal');set(ax1,'YDir','reverse');xlabel(ax1,'Workspace X [mm]');ylabel(ax1,'Workspace Y [mm]');ax2=nexttile;hold(ax2,'on');grid(ax2,'on');axis(ax2,'equal');axis(ax2,'vis3d');view(ax2,[-35,22]);ax3=nexttile([1,2]);hold(ax3,'on');grid(ax3,'on');xlabel(ax3,'Scheduled execution time [s]');
    yyaxis(ax3,'left'); ylabel(ax3,'Joint q [rad]'); jointLines=gobjects(1,context.dof); colors=lines(context.dof); for i=1:context.dof,jointLines(i)=plot(ax3,NaN,NaN,'Color',colors(i,:),'LineStyle','-','LineWidth',1.2,'DisplayName',context.joint_names{i});end
    yyaxis(ax3,'right'); ylabel(ax3,'Execution backlog [s]'); backlogLine=plot(ax3,NaN,NaN,'k--','LineWidth',1.3,'DisplayName','Execution backlog [s]');
    f.UserData=struct('workspace',ax1,'robot',ax2,'joint',ax3,'sourceLine',plot(ax1,NaN,NaN,'bo-','LineWidth',1.0,'MarkerSize',4,'DisplayName','Formal robot targets (post-offset)'),'planLine',plot(ax1,NaN,NaN,'r-','LineWidth',1.3,'DisplayName','Cartesian quintics'),'currentPoint',plot(ax1,NaN,NaN,'ko','MarkerFaceColor','y','DisplayName','Latest formal target'),'backlogLine',backlogLine,'jointLines',jointLines,'visualizer',live_legacy5_visualizer('create',context.model,q,ax2));
    legend(ax1,'Location','best');legend(ax3,'Location','eastoutside');update_workspace_limits(f,task);
end
function update_workspace_limits(f,task),if ~isvalid(f),return,end;ax=f.UserData.workspace;xlim(ax,[0 task.workspace_width_mm]);ylim(ax,[0 task.workspace_height_mm]);end
function update_live_figure(f,context,q,source,planned,executedFK,t,qh,backlog,published,queued,active,state,heartbeats,lateness)
    if ~isvalid(f),return,end
    h=f.UserData;set(h.sourceLine,'XData',source(:,1),'YData',source(:,2));set(h.planLine,'XData',planned(:,1),'YData',planned(:,2));if ~isempty(source),set(h.currentPoint,'XData',source(end,1),'YData',source(end,2));end
    referenceFK=workspace_trace_to_base_live(planned,state.task_plane,600);
    h.visualizer=live_legacy5_visualizer('update',h.visualizer,context.model,q);h.visualizer=live_legacy5_visualizer('trails',h.visualizer,referenceFK,executedFK);f.UserData=h;
    yyaxis(h.joint,'left');for i=1:context.dof,set(h.jointLines(i),'XData',t,'YData',qh(:,i));end
    yyaxis(h.joint,'right');set(h.backlogLine,'XData',t,'YData',backlog);
    maxLate=0;latestLate=0;if ~isempty(lateness),maxLate=max(lateness);latestLate=lateness(end);end
    sourceTiming='source latency=--';if isfinite(state.last_source_latency_ms),sourceTiming=sprintf('source availability latency=%.0f ms',state.last_source_latency_ms);end
    block='';if ~isempty(state.block_reason),block=sprintf(' | block=%s@%g: %s',state.block_message_kind,state.block_source_index,state.block_reason);end
    title(h.workspace,sprintf('formal source=%d | heartbeats=%d | admission=%s%s',published,heartbeats,admission_text_live(state),block));title(h.robot,'Legacy5: black=Cartesian reference | green=executed FK trail');title(h.joint,sprintf('active=%s | q(t) samples=%d | waiting=%d | scheduler late latest/max=%.3f/%.3f s | %s',active_text_live(active),numel(t),queued,latestLate,maxLate,sourceTiming));drawnow limitrate;
end
function s=active_text_live(active),if isempty(active),s='HOLD';else,s=active.kind;end,end
function delete_if_valid(server),try,if isvalid(server),delete(server);end,catch,end,end
