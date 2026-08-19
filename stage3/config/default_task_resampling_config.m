function config = default_task_resampling_config()
%DEFAULT_TASK_RESAMPLING_CONFIG Piecewise-linear task-space resampling.
%
% target_spacing_m is a robot-base task-space distance, not a camera-frame
% rate and not a time parameter.  2 mm is a conservative baseline that can
% be adjusted explicitly by later trajectory experiments.

    config.target_spacing_m = 0.002;
end
