function q = row_configuration(configuration)
%ROW_CONFIGURATION Normalize a Legacy5 configuration to one 1-by-DOF vector.
%
% Stage 3.2 keeps an equivalent helper private to its solver file.  The
% realtime adapter needs the same normalization at its separate module
% boundary, so it owns this tiny explicit copy rather than reaching into a
% private local function.

    if isstruct(configuration)
        q = [configuration.JointPosition];
    else
        q = configuration;
    end
    q = reshape(q, 1, []);
    if ~isnumeric(q) || ~isreal(q) || any(~isfinite(q))
        error('MonoTeach:RealtimeInvalidConfiguration', ...
            'IK/home configuration must be a finite real row vector.');
    end
end
