function trace = replay_compiled_banter_motion(compiled)
%REPLAY_COMPILED_BANTER_MOTION Flatten canonical Task 5/6 q(t) samples.
%   No reinterpolation is performed: segment q_rad/t_s are the execution
%   source of truth.  The caller schedules samples against trace.t_s.

    if ~isstruct(compiled) || ~isfield(compiled, 'continuous_trajectory')
        error('MonoTeach:BanterExecutionTrace', 'Expected a CompiledStyledBanterMotion struct.');
    end
    segments = compiled.continuous_trajectory.segments;
    allT = zeros(0, 1); allQ = zeros(0, size(segments(1).q_rad, 2)); offset = 0.0;
    for index = 1:numel(segments)
        segment = segments(index);
        localT = double(segment.t_s(:));
        if isempty(localT) || localT(1) < -1.0e-10 || any(diff(localT) < -1.0e-10)
            error('MonoTeach:BanterExecutionTrace', 'Segment %d has invalid local time samples.', index);
        end
        if size(segment.q_rad, 1) ~= numel(localT)
            error('MonoTeach:BanterExecutionTrace', 'Segment %d q/t sample count mismatch.', index);
        end
        if index > 1
            % Boundary samples are identical endpoints.  Keep the latter
            % timing boundary but avoid a redundant instant draw/command.
            localT = localT(2:end); q = segment.q_rad(2:end, :);
        else
            q = segment.q_rad;
        end
        allT = [allT; offset + localT]; %#ok<AGROW>
        allQ = [allQ; q]; %#ok<AGROW>
        offset = offset + double(segment.duration_s);
    end
    if isempty(allT) || abs(allT(end) - offset) > 1.0e-8
        error('MonoTeach:BanterExecutionTrace', 'Trace duration does not equal its compiled segments.');
    end
    trace = struct('t_s', allT, 'q_rad', allQ, 'duration_s', offset, ...
        'sample_count', numel(allT), 'final_q_rad', allQ(end, :));
end
