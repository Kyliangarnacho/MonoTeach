function config = default_writing_anchor_diagnostic_config()
%DEFAULT_WRITING_ANCHOR_DIAGNOSTIC_CONFIG Reproducible anchor diagnostic plan.
%
% The 20 deterministic initial configurations are q_anchor, home, and 18
% fixed pseudorandom joint-limit samples. They are reused unchanged for each
% normal and solver, so comparisons are not confounded by different seeds.

    config.random_seed = 7331;
    config.random_seed_count = 18;
    config.solver_algorithms = { ...
        'BFGSGradientProjection', ...
        'LevenbergMarquardt'};

    % Multi-start supplies all seed variation explicitly. Keep GIK's own
    % restart disabled so the diagnostic is repeatable per initial q.
    config.allow_random_restart = false;
end
