function guarded = generate_guarded_minimum_jerk_profile(waypointQRad, timePointsS, config)
%GENERATE_GUARDED_MINIMUM_JERK_PROFILE Minimum jerk with local envelope guard.
%
% This helper never changes waypoint positions or time points.  When the
% unpatched profile exits one joint's adjacent-waypoint envelope, only the
% affected joint's adjacent internal boundary conditions are deterministically
% damped before re-running minjerkpolytraj.

    validate_inputs(waypointQRad, timePointsS, config);
    waypointCount = size(waypointQRad, 1);
    [velocityBC, accelerationBC] = default_boundaries(waypointCount);
    initial = evaluate_profile(waypointQRad, timePointsS, velocityBC, ...
        accelerationBC, config);
    baseVelocity = initial.waypoint_qd_rad_s;
    baseAcceleration = initial.waypoint_qdd_rad_s2;
    % Once a correction is needed, hold the unconstrained min-jerk solution
    % at every untouched interior waypoint.  Leaving those entries as NaN on
    % subsequent calls would re-optimise them globally, which can move an
    % unrelated joint/interval while we are trying to damp one local defect.
    effectiveVelocityBC = baseVelocity';
    effectiveAccelerationBC = baseAcceleration';
    requestedVelocityBC = velocityBC;
    requestedAccelerationBC = accelerationBC;
    velocityLevels = zeros(5, waypointCount);
    accelerationLevels = zeros(5, waypointCount);
    velocityZeroMask = false(5, waypointCount);
    current = initial;
    selectedIteration = 0;
    correctionIteration = 0;

    while current.overshoot.has_overshoot && ...
            correctionIteration < config.max_overshoot_correction_iterations
        [affected, reversal] = affected_boundary_points( ...
            current.overshoot.records, waypointQRad);
        candidates = repmat(candidate_state(), 1, 4);
        % The final candidate is the deterministic local-damping limit:
        % zero V/A only at the endpoints implicated by the evidence.  It is
        % not a global "all internal derivatives are zero" construction.
        modes = [true false; false true; true true; true true];
        dampingFactors = [config.overshoot_boundary_damping_factor, ...
            config.overshoot_boundary_damping_factor, ...
            config.overshoot_boundary_damping_factor, 0.0];
        for modeIndex = 1:size(modes, 1)
            [candidateVelocity, candidateAcceleration, candidateRequestedVelocity, ...
                    candidateRequestedAcceleration, candidateVelocityLevels, ...
                    candidateAccelerationLevels, candidateZeroMask] = damp_boundaries( ...
                effectiveVelocityBC, effectiveAccelerationBC, requestedVelocityBC, ...
                requestedAccelerationBC, baseVelocity, baseAcceleration, velocityLevels, ...
                accelerationLevels, velocityZeroMask, affected, reversal, ...
                dampingFactors(modeIndex), modes(modeIndex, 1), ...
                modes(modeIndex, 2));
            candidates(modeIndex).generated = evaluate_profile(waypointQRad, ...
                timePointsS, candidateVelocity, candidateAcceleration, config);
            candidates(modeIndex).effective_velocity = candidateVelocity;
            candidates(modeIndex).effective_acceleration = candidateAcceleration;
            candidates(modeIndex).requested_velocity = candidateRequestedVelocity;
            candidates(modeIndex).requested_acceleration = candidateRequestedAcceleration;
            candidates(modeIndex).velocity_levels = candidateVelocityLevels;
            candidates(modeIndex).acceleration_levels = candidateAccelerationLevels;
            candidates(modeIndex).zero_mask = candidateZeroMask;
        end
        [candidate, accepted] = best_improving_candidate(candidates, current.overshoot);
        correctionIteration = correctionIteration + 1;
        if ~accepted
            break;
        end
        selectedIteration = correctionIteration;
        current = candidate.generated;
        effectiveVelocityBC = candidate.effective_velocity;
        effectiveAccelerationBC = candidate.effective_acceleration;
        requestedVelocityBC = candidate.requested_velocity;
        requestedAccelerationBC = candidate.requested_acceleration;
        velocityLevels = candidate.velocity_levels;
        accelerationLevels = candidate.acceleration_levels;
        velocityZeroMask = candidate.zero_mask;
    end
    guarded = struct('generated', current, 'plain_generated', initial, ...
        'overshoot_before_patch', initial.overshoot, ...
        'overshoot_after_patch', current.overshoot, ...
        'overshoot_patch_iterations', correctionIteration, ...
        'overshoot_patch_selected_iteration', selectedIteration, ...
        'overshoot_patch_applied', selectedIteration > 0, ...
        'velocity_boundary_condition_rad_s', requestedVelocityBC', ...
        'acceleration_boundary_condition_rad_s2', requestedAccelerationBC', ...
        'velocity_boundary_modified_mask', velocityLevels' > 0 | velocityZeroMask', ...
        'acceleration_boundary_modified_mask', accelerationLevels' > 0, ...
        'velocity_boundary_zeroed_at_reversal_mask', velocityZeroMask');
end


function generated = evaluate_profile(waypointQ, timePoints, velocityBC, accelerationBC, config)

    [~, ~, ~, ~, pp] = minjerkpolytraj(waypointQ', timePoints, 2, ...
        VelocityBoundaryCondition=velocityBC, ...
        AccelerationBoundaryCondition=accelerationBC, ...
        TimeAllocation=false);
    sampleTimes = trajectory_sample_times(timePoints, config.continuous_sample_period_s);
    guardTimes = trajectory_sample_times(timePoints, config.overshoot_guard_sample_period_s);
    generated = struct();
    generated.sample_t_s = sampleTimes(:);
    generated.q_rad = ppval(pp, sampleTimes)';
    generated.qd_rad_s = ppval(fnder(pp, 1), sampleTimes)';
    generated.qdd_rad_s2 = ppval(fnder(pp, 2), sampleTimes)';
    generated.qddd_rad_s3 = ppval(fnder(pp, 3), sampleTimes)';
    generated.waypoint_q_rad = ppval(pp, timePoints)';
    generated.waypoint_qd_rad_s = ppval(fnder(pp, 1), timePoints)';
    generated.waypoint_qdd_rad_s2 = ppval(fnder(pp, 2), timePoints)';
    generated.pp = pp;
    generated.overshoot = detect_joint_interval_overshoot(guardTimes, ...
        ppval(pp, guardTimes)', timePoints(:), waypointQ, ...
        config.overshoot_numerical_epsilon_rad);
end


function [velocityBC, accelerationBC] = default_boundaries(waypointCount)

    velocityBC = nan(5, waypointCount);
    accelerationBC = nan(5, waypointCount);
    velocityBC(:, [1, end]) = 0.0;
    accelerationBC(:, [1, end]) = 0.0;
end


function [affected, reversal] = affected_boundary_points(records, waypointQ)

    waypointCount = size(waypointQ, 1);
    affected = false(5, waypointCount);
    reversal = false(5, waypointCount);
    for recordIndex = 1:numel(records)
        record = records(recordIndex);
        joint = record.joint_index;
        endpoints = [record.interval_index, record.interval_index + 1];
        for waypoint = endpoints
            if waypoint <= 1 || waypoint >= waypointCount
                continue;
            end
            affected(joint, waypoint) = true;
            before = waypointQ(waypoint, joint) - waypointQ(waypoint - 1, joint);
            after = waypointQ(waypoint + 1, joint) - waypointQ(waypoint, joint);
            reversal(joint, waypoint) = reversal(joint, waypoint) || before * after <= 0;
        end
    end
end


function [effectiveVelocityBC, effectiveAccelerationBC, requestedVelocityBC, ...
        requestedAccelerationBC, velocityLevels, accelerationLevels, zeroMask] = ...
        damp_boundaries(effectiveVelocityBC, effectiveAccelerationBC, ...
        requestedVelocityBC, requestedAccelerationBC, baseVelocity, baseAcceleration, ...
        velocityLevels, accelerationLevels, zeroMask, affected, reversal, damping, ...
        dampVelocity, dampAcceleration)

    [jointIndices, waypointIndices] = find(affected);
    for index = 1:numel(jointIndices)
        joint = jointIndices(index);
        waypoint = waypointIndices(index);
        if dampVelocity
            velocityLevels(joint, waypoint) = velocityLevels(joint, waypoint) + 1;
            if reversal(joint, waypoint)
                effectiveVelocityBC(joint, waypoint) = 0.0;
                requestedVelocityBC(joint, waypoint) = 0.0;
                zeroMask(joint, waypoint) = true;
            else
                effectiveVelocityBC(joint, waypoint) = baseVelocity(waypoint, joint) * ...
                    damping ^ velocityLevels(joint, waypoint);
                requestedVelocityBC(joint, waypoint) = effectiveVelocityBC(joint, waypoint);
            end
        end
        if dampAcceleration
            accelerationLevels(joint, waypoint) = accelerationLevels(joint, waypoint) + 1;
            effectiveAccelerationBC(joint, waypoint) = baseAcceleration(waypoint, joint) * ...
                damping ^ accelerationLevels(joint, waypoint);
            requestedAccelerationBC(joint, waypoint) = ...
                effectiveAccelerationBC(joint, waypoint);
        end
    end
end


function better = evidence_is_better(candidate, incumbent)
% Prefer a smaller peak violation; break a numerical tie by record count.
    tolerance = 1e-15;
    better = candidate.max_overshoot_magnitude_rad < ...
        incumbent.max_overshoot_magnitude_rad - tolerance || ...
        (abs(candidate.max_overshoot_magnitude_rad - ...
        incumbent.max_overshoot_magnitude_rad) <= tolerance && ...
        candidate.count < incumbent.count);
end


function [candidate, accepted] = best_improving_candidate(candidates, incumbent)

    candidate = candidate_state();
    accepted = false;
    for index = 1:numel(candidates)
        if ~accepted || evidence_is_better(candidates(index).generated.overshoot, ...
                candidate.generated.overshoot)
            candidate = candidates(index);
            accepted = true;
        end
    end
    accepted = accepted && evidence_is_better(candidate.generated.overshoot, incumbent);
end


function state = candidate_state()

    state = struct('generated', struct('overshoot', struct( ...
        'max_overshoot_magnitude_rad', inf, 'count', inf)), ...
        'effective_velocity', [], 'effective_acceleration', [], ...
        'requested_velocity', [], 'requested_acceleration', [], ...
        'velocity_levels', [], 'acceleration_levels', [], 'zero_mask', []);
end


function times = trajectory_sample_times(timePoints, samplePeriod)

    times = unique([timePoints(1):samplePeriod:timePoints(end), ...
        timePoints, timePoints(end)]);
end


function validate_inputs(waypointQ, timePoints, config)

    required = {'continuous_sample_period_s', 'overshoot_guard_sample_period_s', ...
        'overshoot_numerical_epsilon_rad', ...
        'max_overshoot_correction_iterations', ...
        'overshoot_boundary_damping_factor'};
    if ~isnumeric(waypointQ) || ~isreal(waypointQ) || ...
            size(waypointQ, 1) < 2 || ~isequal(size(waypointQ, 2), 5) || ...
            any(~isfinite(waypointQ), 'all') || ...
            ~isnumeric(timePoints) || ~isreal(timePoints) || ...
            ~isvector(timePoints) || numel(timePoints) ~= size(waypointQ, 1) || ...
            any(~isfinite(timePoints)) || any(diff(timePoints(:)) <= 0) || ...
            ~isstruct(config) || ~all(isfield(config, required)) || ...
            config.continuous_sample_period_s <= 0 || ...
            config.overshoot_guard_sample_period_s <= 0 || ...
            config.overshoot_numerical_epsilon_rad < 0 || ...
            config.max_overshoot_correction_iterations < 0 || ...
            config.max_overshoot_correction_iterations ~= ...
                floor(config.max_overshoot_correction_iterations) || ...
            config.overshoot_boundary_damping_factor <= 0 || ...
            config.overshoot_boundary_damping_factor >= 1
        error('MonoTeach:InvalidMinimumJerkOvershootConfig', ...
            'Overshoot guard requires finite waypoints, increasing times, and deterministic damping settings.');
    end
end
