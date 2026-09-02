function tests = test_banter_task7_execution
%TEST_BANTER_TASK7_EXECUTION Offline Task 7 cache/trace regression checks.
    tests = functiontests(localfunctions);
end

function test_precompiled_library_replays_canonical_safe_samples(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    fixture = fullfile(root, 'outputs', 'banter_task6_styled_plans.json');
    if ~isfile(fixture)
        error('MonoTeach:BanterFixtureRequired', 'Generate outputs/banter_task6_styled_plans.json before MATLAB Task 7 tests.');
    end
    addpath(genpath(fullfile(root, 'stage3')));
    addpath(fullfile(root, 'stage1'));
    addpath(fullfile(root, 'banter', 'matlab'));
    context = load_robot_context('legacy5');
    library = load_banter_execution_library(fixture, context);
    verifyEqual(testCase, numel(library.entries), 12);
    for index = 1:numel(library.entries)
        trace = replay_compiled_banter_motion(library.entries(index).compiled);
        verifyGreaterThan(testCase, trace.sample_count, 1);
        verifyGreaterThan(testCase, trace.duration_s, 0.0);
        verifyLessThanOrEqual(testCase, norm(trace.final_q_rad - context.home_q, inf), 1.0e-8);
        verifyGreaterThanOrEqual(testCase, min(diff(trace.t_s)), -1.0e-10);
    end
end
