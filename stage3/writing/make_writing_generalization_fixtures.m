function fixtures = make_writing_generalization_fixtures()
%MAKE_WRITING_GENERALIZATION_FIXTURES Deterministic single-stroke workspace paths.
%
% Synthetic fixtures and the fixed recorded C920 triangle all use the same
% 190-by-290 mm WorkspaceTrajectory schema contract.  They are fixtures only;
% no candidate geometry, solver, or tolerance is adjusted per path.

    widthMM = 190.0;
    heightMM = 290.0;
    spacingMM = 5.0;
    fixtures = repmat(empty_fixture(), 1, 0);
    fixtures(end + 1) = make_fixture('straight_line', ...
        sample_polyline([45, 145; 145, 145], spacingMM), widthMM, heightMM); %#ok<AGROW>
    fixtures(end + 1) = make_fixture('rectangle_polyline', ...
        sample_polyline([65, 95; 125, 95; 125, 195; 65, 195; 65, 95], ...
        spacingMM), widthMM, heightMM); %#ok<AGROW>
    theta = linspace(-pi/2, pi/2, 25)';
    fixtures(end + 1) = make_fixture('arc_circle_like', ...
        [95 + 55*cos(theta), 145 + 55*sin(theta)], widthMM, heightMM); %#ok<AGROW>
    u = linspace(-1, 1, 31)';
    fixtures(end + 1) = make_fixture('s_curve', ...
        [95 + 55*u, 145 + 48*sin(pi*u)], widthMM, heightMM); %#ok<AGROW>
    fixtures(end + 1) = make_fixture('zig_zag', ...
        sample_polyline([45, 90; 75, 130; 45, 170; 75, 210; 45, 250], ...
        spacingMM), widthMM, heightMM); %#ok<AGROW>
    fixtures(end + 1) = recorded_triangle_fixture(widthMM, heightMM); %#ok<AGROW>
end


function fixture = recorded_triangle_fixture(widthMM, heightMM)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    files = dir(fullfile(root, 'data', 'workspace_trajectories', ...
        '*_workspace_2d_*.json'));
    if numel(files) ~= 1
        error('MonoTeach:AmbiguousBenchmarkTriangleFixture', ...
            'Expected exactly one fixed recorded triangle WorkspaceTrajectory.');
    end
    trajectory = load_workspace_trajectory(fullfile(files(1).folder, files(1).name));
    if trajectory.metadata.width_mm ~= widthMM || ...
            trajectory.metadata.height_mm ~= heightMM
        error('MonoTeach:BenchmarkWorkspaceContractMismatch', ...
            'Recorded triangle must share the 190-by-290 mm workspace contract.');
    end
    fixture = struct('name', 'recorded_triangle', ...
        'workspace_trajectory', trajectory, 'fixture_kind', 'recorded');
end


function fixture = make_fixture(name, pointsMM, widthMM, heightMM)
    if any(pointsMM(:,1) < 0 | pointsMM(:,1) > widthMM | ...
            pointsMM(:,2) < 0 | pointsMM(:,2) > heightMM)
        error('MonoTeach:BenchmarkFixtureOutsideWorkspace', ...
            'Fixture %s must remain inside the shared Workspace contract.', name);
    end
    samples = repmat(empty_workspace_sample(), 1, size(pointsMM, 1));
    for index = 1:size(pointsMM, 1)
        samples(index) = struct('t_ms', 100.0 * (index - 1), ...
            'valid', true, 'x_mm', pointsMM(index,1), 'y_mm', pointsMM(index,2), ...
            'inside_workspace', true, 'invalid_reason', '');
    end
    trajectory = struct('schema_version', '1.0', ...
        'metadata', struct('source_trajectory_id', ['benchmark_' name], ...
        'workspace_calibration_id', 'benchmark_workspace_contract', ...
        'coordinate_frame', 'workspace_2d', 'width_mm', widthMM, ...
        'height_mm', heightMM), 'samples', samples);
    validate_workspace_trajectory(trajectory);
    fixture = struct('name', name, 'workspace_trajectory', trajectory, ...
        'fixture_kind', 'synthetic');
end


function points = sample_polyline(vertices, spacingMM)
    points = vertices(1, :);
    for index = 1:size(vertices, 1)-1
        startPoint = vertices(index, :);
        endPoint = vertices(index+1, :);
        lengthMM = norm(endPoint - startPoint);
        count = max(1, ceil(lengthMM / spacingMM));
        ratio = (1:count)' / count;
        points = [points; (1-ratio)*startPoint + ratio*endPoint]; %#ok<AGROW>
    end
end


function fixture = empty_fixture()
    fixture = struct('name', '', 'workspace_trajectory', struct(), ...
        'fixture_kind', '');
end


function sample = empty_workspace_sample()
    sample = struct('t_ms', NaN, 'valid', true, 'x_mm', NaN, 'y_mm', NaN, ...
        'inside_workspace', true, 'invalid_reason', '');
end
