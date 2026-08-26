function report = verify_replay_trace_execution(tracePath, config)
%VERIFY_REPLAY_TRACE_EXECUTION Check completion evidence after the final q sample.
%
% The executor records a sample at a chunk's exact endpoint before it releases
% that chunk from its internal active slot.  This verifier deliberately checks
% the externally meaningful condition instead: every Cartesian segment became
% ready and the FIFO had no waiting chunk at the last recorded controller tick.

    if nargin < 2
        config = [];
    end

    run = run_replay_trace_execution(tracePath, config);
    execution = run.execution;
    trace = run.trace;
    report = struct();
    report.run = run;
    report.all_segments_became_ready = execution.segments_ready_count(end) == ...
        numel(trace.planned_segments);
    report.waiting_q_queue_empty = execution.queued_chunk_count(end) == 0;
    report.final_sample_is_last_segment_endpoint = isfinite( ...
        execution.active_segment_index(end));
    report.drained_after_final_endpoint = report.all_segments_became_ready && ...
        report.waiting_q_queue_empty;
    if ~report.drained_after_final_endpoint
        error('MonoTeach:ReplayExecutionNotDrained', ...
            'Not all q(t) chunks reached the controller by the end of simulation.');
    end
end
