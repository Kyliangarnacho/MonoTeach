function trajectory = load_workspace_trajectory(jsonPath)
%LOAD_WORKSPACE_TRAJECTORY Load one Stage 2.3 workspace trajectory JSON file.
%
% Input:
%   jsonPath    - path to a schema_version 1.0 workspace trajectory JSON
%
% Output:
%   trajectory  - decoded JSON struct with metadata and samples unchanged
%
% The loader intentionally does not infer defaults or transform coordinates.
% It only reads the artifact and validates the current Stage 2.3 contract.

    if nargin ~= 1
        error( ...
            'MonoTeach:InvalidArgumentCount', ...
            'load_workspace_trajectory requires exactly one JSON path.');
    end

    try
        jsonText = fileread(jsonPath);
    catch readError
        error( ...
            'MonoTeach:WorkspaceTrajectoryReadFailed', ...
            'Cannot read workspace trajectory JSON: %s', ...
            readError.message);
    end

    try
        trajectory = jsondecode(jsonText);
    catch decodeError
        error( ...
            'MonoTeach:WorkspaceTrajectoryMalformedJson', ...
            'Malformed workspace trajectory JSON: %s', ...
            decodeError.message);
    end

    validate_workspace_trajectory(trajectory);
end
