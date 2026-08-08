%% MonoTeach Stage 1
% IK Motion Demo with visible simplified robot geometry.
%
% Flow:
%   Build robot -> create reachable target -> numerical IK -> FK check
%   -> smooth joint-space interpolation -> visible robot animation
%   -> end-effector path

clear;
clc;
close all;

%% 1. Build robot

robot = build_legacy_robot();
qStart = homeConfiguration(robot);


%% 2. Create a known reachable target

qTargetReference = deg2rad([30, 60, -30, 120, 45]);

TTarget = getTransform( ...
    robot, ...
    qTargetReference, ...
    'body5');


%% 3. Solve numerical IK

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1, 1, 1, 1, 1, 1];

[qIK, solInfo] = ik( ...
    'body5', ...
    TTarget, ...
    weights, ...
    qStart);


%% 4. FK back-substitution verification

TRecovered = getTransform(robot, qIK, 'body5');

positionError = norm( ...
    TRecovered(1:3, 4) - TTarget(1:3, 4));

transformError = max( ...
    abs(TRecovered - TTarget), ...
    [], ...
    'all');

fprintf('====================================\n');
fprintf('MonoTeach Stage 1 IK Motion Demo\n');
fprintf('====================================\n');
fprintf('IK status: %s\n', solInfo.Status);
fprintf('Position error : %.3e m\n', positionError);
fprintf('Transform error: %.3e\n', transformError);
fprintf('IK solution (deg):\n');
disp(rad2deg(qIK));


%% 5. Generate a smooth joint-space motion

numSteps = 80;
trajectory = zeros(numSteps, numel(qStart));

for i = 1:numSteps
    s = (i - 1) / (numSteps - 1);

    % Cubic smoothstep: zero slope at start and end.
    sSmooth = 3*s^2 - 2*s^3;

    trajectory(i, :) = ...
        qStart + sSmooth * (qIK - qStart);
end


%% 6. Pre-compute end-effector path

eePath = zeros(numSteps, 3);

for i = 1:numSteps
    T = getTransform(robot, trajectory(i, :), 'body5');
    eePath(i, :) = T(1:3, 4)';
end


%% 7. Animate simplified visible robot

figure;
ax = axes;

for i = 1:numSteps
    cla(ax);

    q = trajectory(i, :);

    % Draw visible links and joint centers using actual frame origins.
    plot_legacy_robot(robot, q, ax);

    hold(ax, 'on');

    % End-effector trajectory up to current frame.
    plot3( ...
        ax, ...
        eePath(1:i, 1), ...
        eePath(1:i, 2), ...
        eePath(1:i, 3), ...
        'LineWidth', 2);

    % Target position.
    plot3( ...
        ax, ...
        TTarget(1, 4), ...
        TTarget(2, 4), ...
        TTarget(3, 4), ...
        'o', ...
        'MarkerSize', 10, ...
        'LineWidth', 2);

    title(ax, 'MonoTeach Stage 1 - IK Motion');

    % Fixed view prevents frame-to-frame zoom flicker.
    xlim(ax, [-0.35, 0.35]);
    ylim(ax, [-0.35, 0.35]);
    zlim(ax, [-0.05, 0.45]);

    drawnow;
end

fprintf('\nDemo completed.\n');
