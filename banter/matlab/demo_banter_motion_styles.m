function demo_banter_motion_styles(fixturePath, behaviorName, playbackRate, renderRateHz)
%DEMO_BANTER_MOTION_STYLES Offline Task 6 timing-style gallery for Legacy5.

    if nargin < 1 || isempty(fixturePath)
        error('MonoTeach:BanterFixtureRequired', ...
            ['Generate a JSON fixture first, for example: python -m ' ...
            'banter.export_task6_styled_plans outputs/banter_task6_styled_plans.json']);
    end
    if nargin < 2 || isempty(behaviorName), behaviorName = ""; end
    if nargin < 3 || isempty(playbackRate), playbackRate = 1.0; end
    if nargin < 4 || isempty(renderRateHz), renderRateHz = 25.0; end
    if ~isscalar(playbackRate) || ~isfinite(playbackRate) || playbackRate <= 0
        error('MonoTeach:BanterPlaybackRate', 'playbackRate must be finite and positive.');
    end
    if ~isscalar(renderRateHz) || ~isfinite(renderRateHz) || renderRateHz <= 0
        error('MonoTeach:BanterRenderRate', 'renderRateHz must be finite and positive.');
    end
    payload = jsondecode(fileread(fixturePath));
    if ~isfield(payload, 'plans') || ~isstruct(payload.plans)
        error('MonoTeach:BanterFixture', 'Fixture must contain a plans array.');
    end
    plans = payload.plans;
    if strlength(string(behaviorName)) > 0
        keep = arrayfun(@(item) strcmp(string(item.behavior_name), string(behaviorName)), plans);
        plans = plans(keep);
        if isempty(plans)
            error('MonoTeach:BanterBehaviorFilter', 'Fixture has no behavior named %s.', char(string(behaviorName)));
        end
    end
    context = load_robot_context('legacy5');
    figureHandle = figure('Name', 'MonoTeach Banter Task 6 Motion Style Gallery');
    ax = axes(figureHandle); grid(ax, 'on'); view(ax, 3); axis(ax, 'equal');
    visualizer = live_legacy5_visualizer('create', context.model, context.home_q, ax);
    for index = 1:numel(plans)
        compiled = compile_styled_banter_motion_plan(plans(index), context);
        title(ax, sprintf('%s : %s | %s | requested %.2fs / resolved %.2fs / actual %.2fs', ...
            char(string(plans(index).behavior_name)), compiled.variant, compiled.macro_name, ...
            compiled.summary.requested_styled_duration_s, ...
            compiled.summary.resolved_timing_duration_s, ...
            compiled.summary.execution_duration_s));
        for segmentIndex = 1:numel(compiled.continuous_trajectory.segments)
            segment = compiled.continuous_trajectory.segments(segmentIndex);
            firstT = segment.t_s(1); wall = tic;
            % TOPP-RA produces a dense safe trajectory.  Render only a
            % human-visible subset, but schedule those samples against the
            % original segment clock. A stationary HOLD needs only its two
            % endpoints, so repeated rendering cannot lengthen the pause.
            if strcmp(string(compiled.segment_groups(segmentIndex).kind), "HOLD")
                renderIndices = unique([1, numel(segment.t_s)]);
            else
                renderIndices = render_sample_indices(segment.t_s, playbackRate, renderRateHz);
            end
            for sampleIndex = renderIndices
                desired = (segment.t_s(sampleIndex) - firstT) / playbackRate;
                pause(max(0.0, desired - toc(wall)));
                visualizer = live_legacy5_visualizer('update', visualizer, ...
                    context.model, segment.q_rad(sampleIndex, :));
                drawnow limitrate;
            end
        end
        pause(0.35);
    end
end

function indices = render_sample_indices(t_s, playbackRate, renderRateHz)
    indices = 1;
    maximumTrajectoryIntervalS = playbackRate / renderRateHz;
    lastDrawT = t_s(1);
    for index = 2:numel(t_s) - 1
        if t_s(index) - lastDrawT >= maximumTrajectoryIntervalS
            indices(end + 1) = index; %#ok<AGROW>
            lastDrawT = t_s(index);
        end
    end
    if indices(end) ~= numel(t_s)
        indices(end + 1) = numel(t_s); %#ok<AGROW>
    end
end
