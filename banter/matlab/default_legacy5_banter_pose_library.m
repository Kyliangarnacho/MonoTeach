function poses = default_legacy5_banter_pose_library(robotContext)
%DEFAULT_LEGACY5_BANTER_POSE_LIBRARY Conservative named Legacy5 expression poses.
%   These are simulation-only nominal joint poses. They are deliberately
%   separate from Gesture/Behavior semantics and contain no timing or style.

    if nargin < 1 || isempty(robotContext)
        robotContext = load_robot_context('legacy5');
    end
    if ~isstruct(robotContext) || ~isscalar(robotContext) || ...
            ~all(isfield(robotContext, {'dof', 'home_q', 'joint_limits'})) || ...
            robotContext.dof ~= 5
        error('MonoTeach:BanterPoseLibraryContext', ...
            'Banter pose library requires the Legacy5 RobotContext.');
    end

    poses = struct();
    poses.NEUTRAL = robotContext.home_q;
    poses.AIM = [0.20, 1.35, -0.30, 1.65, 0.20];
    poses.JAB_EXTEND = [0.30, 1.15, -0.55, 1.95, 0.35];
    poses.SALUTE_RAISE = [-0.35, 1.85, 0.45, 1.45, -0.25];
    poses.SALUTE_OUT = [-0.55, 1.75, 0.65, 1.35, -0.45];
    poses.WAVE_LEFT = [-0.45, 1.55, 0.45, 1.55, -0.55];
    poses.WAVE_RIGHT = [0.45, 1.55, -0.45, 1.55, 0.55];
    poses.ACK_UP = [0.00, 1.90, 0.00, 1.20, 0.00];
    poses.RECOIL = [0.00, 1.25, 0.30, 1.80, 0.00];
    poses.DISMISS_LEFT = [-0.65, 1.45, 0.55, 1.60, -0.40];
    poses.DISMISS_RIGHT = [0.45, 1.45, -0.45, 1.60, 0.40];
    poses.POWER_HIGH = [0.00, 2.00, 0.00, 1.00, 0.00];
    poses.FLOURISH_LOW = [-0.35, 1.20, 0.30, 1.90, -0.40];
    poses.FLOURISH_HIGH = [0.25, 2.05, -0.35, 1.10, 0.45];
    poses.PROUD_IN = [0.00, 1.35, 0.10, 1.80, 0.00];
    poses.PROUD_UP = [0.00, 1.70, -0.15, 1.40, 0.00];
    poses.HYPE_DOWN = [0.00, 1.35, 0.40, 1.90, 0.20];
    poses.HYPE_UP = [0.00, 1.85, -0.25, 1.25, -0.20];

    names = fieldnames(poses);
    for index = 1:numel(names)
        q = poses.(names{index});
        if ~isequal(size(q), [1, robotContext.dof]) || any(~isfinite(q)) || ...
                any(q < robotContext.joint_limits(:, 1)' - 1.0e-12) || ...
                any(q > robotContext.joint_limits(:, 2)' + 1.0e-12)
            error('MonoTeach:BanterPoseLibraryLimits', ...
                'Pose %s must be finite and inside Legacy5 joint limits.', names{index});
        end
    end
end
