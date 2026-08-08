function robot = build_legacy_robot()
%BUILD_LEGACY_ROBOT Build MonoTeach Legacy 5DOF robot model.
%
% Output:
%   robot - MATLAB rigidBodyTree model of the Legacy 5DOF arm
%
% Convention:
%   Modified DH
%
% Units:
%   length -> meter
%   angle  -> radian

    %% 1. 创建整台机器人
    robot = rigidBodyTree( ...
        'DataFormat', 'row', ...
        'MaxNumBodies', 5);

    robot.Gravity = [0 0 -9.81];


    %% 2. Stage 0 Modified DH 参数
    % 每一行：
    % [a(i-1), alpha(i-1), d(i), theta]
    %
    % 注意：
    % MATLAB 中角度使用 rad
    % revolute joint 的 theta 由关节角 q 决定，
    % 所以这里 theta 写 0 即可。

    mdhParams = [
        0.000,   0,      0.068, 0;
        0.000,   pi/2,   0.000, 0;
        0.085,   pi,     0.000, 0;
        0.080,   0,      0.000, 0;
        0.000,   pi/2,   0.105, 0
    ];


    %% 3. Stage 0 关节限位（deg）
    jointLimitsDeg = [
        -120, 120;
           0, 180;
        -120, 120;
         -30, 210;
        -120, 120
    ];


    %% 4. Stage 0 默认姿态（deg）
    homeDeg = [
         0;
        90;
         0;
        90;
         0
    ];


    %% 5. 依次创建 J1 ~ J5

    parentName = 'base';

    for i = 1:5

        % 创建第 i 个刚体
        bodyName = sprintf('body%d', i);
        body = rigidBody(bodyName);

        % 创建第 i 个旋转关节
        jointName = sprintf('joint%d', i);
        joint = rigidBodyJoint(jointName, 'revolute');

        % 设置 Modified DH 几何关系
        setFixedTransform( ...
            joint, ...
            mdhParams(i, :), ...
            'mdh');

        % 设置关节限位
        joint.PositionLimits = ...
            deg2rad(jointLimitsDeg(i, :));

        % 设置默认姿态
        joint.HomePosition = ...
            deg2rad(homeDeg(i));

        % 把 joint 装到 body
        body.Joint = joint;

        % 把当前 body 接到上一节 body
        addBody(robot, body, parentName);

        % 下一节的父节点就是当前 body
        parentName = bodyName;

    end

end