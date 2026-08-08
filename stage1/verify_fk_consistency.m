%% MonoTeach Stage 1
% Python <-> MATLAB FK consistency verification

clear;
clc;

format long g;


%% 1. Build MATLAB robot

robot = build_legacy_robot();


%% 2. Load Python reference data

jsonText = fileread( ...
    fullfile('data', 'python_fk_reference.json'));

reference = jsondecode(jsonText);


%% 3. Compare all test cases

tolerance = 1e-10;

allPassed = true;

fprintf('========================================\n');
fprintf('MonoTeach FK Consistency Verification\n');
fprintf('========================================\n');


for i = 1:numel(reference)

    name = reference(i).name;

    qDeg = ...
        reference(i).joint_angles_deg(:)';

    qRad = deg2rad(qDeg);


    % Python reference

    T_python = ...
        reference(i).transform;

    p_python = ...
        reference(i).position_m(:)';


    % MATLAB FK

    T_matlab = getTransform( ...
        robot, ...
        qRad, ...
        'body5');

    p_matlab = ...
        T_matlab(1:3, 4)';


    % Errors

    positionError = ...
        max(abs(p_matlab - p_python));

    transformError = ...
        max(abs(T_matlab - T_python), [], 'all');


    fprintf('\n[%s]\n', name);

    fprintf('q(deg): ');
    fprintf('%8.3f ', qDeg);
    fprintf('\n');

    fprintf( ...
        'Position max error : %.3e m\n', ...
        positionError);

    fprintf( ...
        'Transform max error: %.3e\n', ...
        transformError);


    if transformError <= tolerance

        fprintf('Result: PASS\n');

    else

        fprintf('Result: FAIL\n');

        allPassed = false;

    end

end


%% 4. Final result

fprintf('\n========================================\n');

if allPassed

    fprintf('FINAL RESULT: PASS\n');

else

    fprintf('FINAL RESULT: FAIL\n');

end

fprintf('========================================\n');