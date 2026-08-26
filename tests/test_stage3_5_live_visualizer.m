function test_stage3_5_live_visualizer()
%TEST_STAGE3_5_LIVE_VISUALIZER Keep the live view at Stage 3.4 visibility.
%
% ``show(rigidBodyTree)`` is intentionally not the assertion: Legacy5 has no
% mesh to show.  The live renderer must expose base + five FK origins, five
% readable joint labels, and the three axes of every joint frame.

    context = load_robot_context('legacy5');
    figureHandle = figure('Visible', 'off', 'Color', 'w');
    ax = axes(figureHandle); %#ok<LAXES>
    visualizer = live_legacy5_visualizer('create', context.model, context.home_q, ax);
    visualizer = live_legacy5_visualizer('update', visualizer, context.model, context.home_q);

    assert(numel(visualizer.markers) == context.model.NumBodies + 1, ...
        'Live renderer must show base plus every Legacy5 body origin.');
    assert(numel(visualizer.labels) == context.model.NumBodies + 1, ...
        'Live renderer must include base and one readable label per joint.');
    assert(isequal(size(visualizer.frames), [context.model.NumBodies, 3]), ...
        'Live renderer must include local X/Y/Z frames for every joint.');
    assert(isgraphics(visualizer.referenceTrail) && isgraphics(visualizer.executedTrail) && ...
        isgraphics(visualizer.endEffector), ...
        'Live renderer must retain desired Cartesian and executed-FK trail handles.');
    reference = [0.10 0.00 0.20; 0.11 0.00 0.21];
    executed = [0.10 0.00 0.20; 0.105 0.00 0.205];
    visualizer = live_legacy5_visualizer('trails', visualizer, reference, executed);
    referenceX = get(visualizer.referenceTrail, 'XData');
    executedZ = get(visualizer.executedTrail, 'ZData');
    assert(isequal(referenceX(:), reference(:,1)) && ...
        isequal(executedZ(:), executed(:,3)), ...
        'Trail update must preserve separately visible reference and FK facts.');
    assert(all(isgraphics(visualizer.markers)) && all(isgraphics(visualizer.labels)) && ...
        all(isgraphics(visualizer.frames), 'all'), ...
        'Live robot graphics handles must remain valid after an update.');
    close(figureHandle);

    % This is intentionally a display-only convention.  The H calibration and
    % all workspace/IK numbers retain their existing mathematical frame; only
    % the live operator plot places y=0 at the physical paper's top edge.
    executorText = fileread(which('run_live_tcp_executor'));
    assert(contains(executorText, "set(ax1,'YDir','reverse')"), ...
        'Live workspace display must use a top-left visual origin.');
    assert(contains(executorText, 'qd0/scale') && contains(executorText, 'qdd0/scale^2') && ...
        contains(executorText, 'qd1/scale') && contains(executorText, 'qdd1/scale^2'), ...
        'Live retiming must uniformly scale endpoint qd/qdd with duration.');
    assert(contains(executorText, "elseif isfield(payload,'source_end_index')"), ...
        'A planned-segment failure must report a real source index, not NaN.');
end
