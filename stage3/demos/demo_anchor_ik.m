%% MonoTeach Stage 3.2A
% Minimal Position-Only IK baseline for the single robot_anchor target.
% This demo is intentionally not a segment/trajectory solver or executor.

clear;
clc;

stage3Dir = fileparts(fileparts(mfilename('fullpath')));
repoDir = fileparts(stage3Dir);
addpath(genpath(stage3Dir));
addpath(fullfile(repoDir, 'stage1'));

robot = build_legacy_robot();
taskPlaneConfig = default_task_plane_config();
ikConfig = default_ik_config();
result = solve_anchor_ik(robot, taskPlaneConfig, ikConfig);

fprintf('============================================\n');
fprintf('MonoTeach Stage 3.2A Anchor IK Baseline\n');
fprintf('============================================\n');
fprintf('End effector          : %s\n', ikConfig.end_effector);
fprintf('IK weights            : %s\n', mat2str(ikConfig.weights));
fprintf('Target XYZ [m]        : [%.9f %.9f %.9f]\n', result.target_xyz_m);
fprintf('Seed q [rad]          : %s\n', mat2str(result.seed_q, 9));
fprintf('Seed q [deg]          : %s\n', mat2str(rad2deg(result.seed_q), 6));
fprintf('Solver status         : %s\n', char(string(result.solver_status)));
fprintf('Pose error norm       : %.9e\n', result.pose_error_norm);
fprintf('q anchor [rad]        : %s\n', mat2str(result.q, 9));
fprintf('q anchor [deg]        : %s\n', mat2str(rad2deg(result.q), 6));
fprintf('FK XYZ [m]            : [%.9f %.9f %.9f]\n', result.fk_xyz_m);
fprintf('Position error [m]    : %.9e\n', result.position_error_m);
fprintf('Position tolerance [m]: %.9e\n', result.position_error_tolerance_m);
fprintf('Within position tol.  : %d\n', result.within_position_tolerance);
fprintf('Within joint limits   : %d\n', result.within_joint_limits);
fprintf('Joint margin [rad]    : %s\n', mat2str(result.joint_limit_margin, 9));
fprintf('============================================\n');
