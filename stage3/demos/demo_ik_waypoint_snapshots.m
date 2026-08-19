function demo = demo_ik_waypoint_snapshots(jsonPath)
%DEMO_IK_WAYPOINT_SNAPSHOTS Show three static Legacy robot IK waypoints.
%
% Input:
%   jsonPath - one real Stage 2.3 WorkspaceTrajectory JSON artifact
%
% Output:
%   demo - the existing continuous-IK derived data plus selected waypoint
%          provenance and a figure handle.
%
% The start, middle, and end are selected from the longest IK-success
% subsegment, rather than from the raw input. Therefore every displayed q is
% an accepted, FK-back-checked waypoint. This is deliberately three static
% poses, not a timed robot animation or an execution command.

    if nargin ~= 1 || ~(ischar(jsonPath) || ...
            (isstring(jsonPath) && isscalar(jsonPath)))
        error( ...
            'MonoTeach:InvalidWaypointSnapshotsInput', ...
            'demo_ik_waypoint_snapshots requires one JSON path.');
    end

    stage3Dir = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(stage3Dir);
    addpath(genpath(stage3Dir));
    addpath(fullfile(repoRoot, 'stage1'));

    % Reuse the unchanged Stage 3.1 -> Stage 3.2 pipeline. This retains any
    % failure barriers; no missing point is repaired merely for display.
    continuousDemo = demo_continuous_ik(char(string(jsonPath)));
    robot = build_legacy_robot();
    longestSegmentIndex = longest_success_segment_index( ...
        continuousDemo.ik_result_set.segments);
    successSegment = continuousDemo.ik_result_set.segments( ...
        longestSegmentIndex);
    sampleIndices = [1, ceil(numel(successSegment.results) / 2), ...
        numel(successSegment.results)];
    labels = {'Start', 'Middle', 'End'};

    figureSnapshots = figure( ...
        'Name', 'MonoTeach Stage 3.2C IK Waypoint Snapshots', ...
        'Position', [100, 100, 1500, 560]);
    layout = tiledlayout(figureSnapshots, 1, 3, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(layout, sprintf( ...
        'Stage 3.2C Static Legacy Robot Poses (IK-success subsegment %d)', ...
        longestSegmentIndex));

    selectedResults = successSegment.results(sampleIndices);

    for i = 1:numel(selectedResults)
        result = selectedResults(i);
        ax = nexttile(layout, i);
        plot_legacy_robot(robot, result.q, ax);
        title(ax, sprintf( ...
            '%s: source %d\nq [rad] = %s', ...
            labels{i}, ...
            result.source_index, ...
            mat2str(result.q, 4)), ...
            'Interpreter', 'none');
    end

    demo = continuousDemo;
    demo.longest_success_segment_index = longestSegmentIndex;
    demo.snapshot_sample_indices = sampleIndices;
    demo.snapshot_results = selectedResults;
    demo.figure_waypoint_snapshots = figureSnapshots;
end


function index = longest_success_segment_index(successSegments)

    if isempty(successSegments)
        error( ...
            'MonoTeach:NoIKSuccessWaypointSnapshots', ...
            'Cannot display snapshots because the IKResultSet has no successes.');
    end

    pointCounts = arrayfun( ...
        @(segment) numel(segment.results), ...
        successSegments);
    [~, index] = max(pointCounts);
end
