%% MonoTeach Stage 3.1A
% Minimal read-only workspace trajectory loader demo.

clear;
clc;

stage3Dir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(stage3Dir);

addpath(stage3Dir);

trajectoryFiles = dir(fullfile( ...
    repoRoot, ...
    'data', ...
    'workspace_trajectories', ...
    '*.json'));

if isempty(trajectoryFiles)
    error( ...
        'MonoTeach:MissingWorkspaceTrajectory', ...
        'No workspace trajectory JSON files were found under data/workspace_trajectories.');
end

jsonPath = fullfile( ...
    trajectoryFiles(1).folder, ...
    trajectoryFiles(1).name);

trajectory = load_workspace_trajectory(jsonPath);
samples = trajectory.samples;

sampleCount = numel(samples);

if sampleCount == 0
    validCount = 0;
    insideCount = 0;
else
    validCount = sum([samples.valid]);
    insideCount = sum([samples.inside_workspace]);
end

fprintf('============================================\n');
fprintf('MonoTeach Stage 3.1A Workspace JSON Loader\n');
fprintf('============================================\n');
fprintf('JSON file   : %s\n', jsonPath);
fprintf('Sample count: %d\n', sampleCount);
fprintf('Valid count : %d\n', validCount);
fprintf('Inside count: %d\n', insideCount);
