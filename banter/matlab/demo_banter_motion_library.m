function demo_banter_motion_library(fixturePath, playbackRate)
%DEMO_BANTER_MOTION_LIBRARY Offline Task 5 Legacy5 motion-gallery acceptance.

    if nargin < 1 || isempty(fixturePath)
        error('MonoTeach:BanterFixtureRequired', ...
            ['Generate a JSON fixture first, for example: python -m ' ...
            'banter.export_task5_motion_plans outputs/banter_task5_plans.json']);
    end
    if nargin < 2 || isempty(playbackRate), playbackRate = 1.0; end
    if ~isscalar(playbackRate) || ~isfinite(playbackRate) || playbackRate <= 0
        error('MonoTeach:BanterPlaybackRate', 'playbackRate must be finite and positive.');
    end
    payload = jsondecode(fileread(fixturePath));
    if ~isfield(payload, 'plans') || ~isstruct(payload.plans)
        error('MonoTeach:BanterFixture', 'Fixture must contain a plans array.');
    end
    context = load_robot_context('legacy5');
    figureHandle = figure('Name', 'MonoTeach Banter Task 5 Motion Gallery');
    ax = axes(figureHandle); grid(ax, 'on'); view(ax, 3); axis(ax, 'equal');
    % Create the Stage 3 FK skeleton once.  Rebuilding rigidBodyTree/show
    % per sample resets the axes and can enter MATLAB's toolbar/pan setup.
    visualizer = live_legacy5_visualizer('create', context.model, context.home_q, ax);
    for index = 1:numel(payload.plans)
        compiled = compile_banter_motion_plan(payload.plans(index), context);
        title(ax, sprintf('%s : %s', compiled.macro_name, compiled.variant));
        for segmentIndex = 1:numel(compiled.continuous_trajectory.segments)
            segment = compiled.continuous_trajectory.segments(segmentIndex);
            firstT = segment.t_s(1); wall = tic;
            for sampleIndex = 1:numel(segment.t_s)
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
