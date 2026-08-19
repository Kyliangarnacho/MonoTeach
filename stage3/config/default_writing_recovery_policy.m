function policy = default_writing_recovery_policy(policyName)
%DEFAULT_WRITING_RECOVERY_POLICY Deterministic writing-IK seed order.
%
% The legacy policy is retained only for controlled A/B diagnosis.  Neither
% policy enables random restart or changes any formal acceptance tolerance.

    if nargin < 1 || isempty(policyName)
        policyName = 'position_only_recovery';
    end
    policyName = char(string(policyName));

    switch policyName
        case 'legacy_previous_then_center'
            seedSources = {'previous_success_q', ...
                'candidate_center_writing_q'};
        case 'position_only_recovery'
            seedSources = {'previous_success_q', ...
                'same_source_position_only_q', ...
                'candidate_center_writing_q', ...
                'home_configuration'};
        otherwise
            error('MonoTeach:UnknownWritingRecoveryPolicy', ...
                'Unsupported Writing IK recovery policy: %s', policyName);
    end

    policy = struct();
    policy.name = policyName;
    policy.seed_sources = seedSources;
    policy.duplicate_seed_tolerance_rad = 1.0e-12;
end
