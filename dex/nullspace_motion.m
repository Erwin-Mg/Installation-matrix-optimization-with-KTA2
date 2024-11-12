% 清除工作区和命令窗口
clear; clc; close all;

% 定义机械臂的 DH 参数
%          theta  d/Z      a/X          alpha
T(1)=Link([0      0.09744*100   0          0    0],'modified');
T(2)=Link([0      0         0          pi/2   0],'modified');
T(2).offset=pi;
T(3)=Link([0      0.39911*100    0     pi/2    0],'modified');
T(4)=Link([0      0   0           pi/2    0],'modified');
T(4).offset=pi/2;
T(5)=Link([0      0         0.44284*100     pi/2    0],'modified');
T(5).offset=pi/2;
T(6)=Link([0        0.1344*100    0         pi/2   0 ],'modified');
T(6).offset=pi/2;
T(7)=Link([0      0   0           pi/2    0],'modified');
T(7).offset=pi/2;
robot = SerialLink(T, 'name', 'Arm1');
robot.tool.t=[0.05 0 0]';

% 设置初始关节角度
I = eye(7);
q0 = randn(1, 7);

% 检查雅可比矩阵在初始位置
J0 = robot.jacob0(q0);
% 计算雅各比矩阵伪逆
J0P = pinv(J0);
disp('Jacobian at initial position:');
disp(J0);

% 计算最小奇异值
singular_values = svd(J0);
min_singular_value = min(singular_values);

% 记录初始末端位置
initial_end_effector_pos = robot.fkine(q0).t';

% 定义零空间运动的幅度
num_steps = 10000;

% 初始化关节角度
q_plus = q0;
q_minus = q0;

% 可视化零空间运动并验证末端位置
for i = 1:num_steps
    if any(abs(q_plus) >= pi) || any(abs(q_minus) >= pi)
        disp('A joint angle reached its limit.');
        break;
    end

    % 计算当前雅可比矩阵和伪逆
    J0_plus = robot.jacob0(q_plus);
    J0_minus = robot.jacob0(q_minus);
    J0P_plus = pinv(J0_plus);
    J0P_minus = pinv(J0_minus);
    
    % 计算零空间
    null_space_plus = null(J0_plus);
    null_space_minus = null(J0_minus);
    
    % 更新关节角度
    q_plus = q_plus + null_space_plus' * 0.001 * randn(size(null_space_plus, 2), 1);
    q_minus = q_minus - null_space_minus' * 0.001 * randn(size(null_space_plus, 2), 1);
    
    % 记录关节角度和末端位置
    q_traj_plus(i,:) = q_plus;
    q_traj_minus(i,:) = q_minus;
    end_effector_pos_plus(i,:) = robot.fkine(q_plus).t';
    end_effector_pos_minus(i,:) = robot.fkine(q_minus).t';
    
    % 计算操控性
    mu_plus(i) = sqrt(det(J0_plus * J0_plus'));
    mu_minus(i) = sqrt(det(J0_minus * J0_minus'));
end
    manu= (mean(mu_plus)+mean(mu_minus))/2;
% 反转 q_minus 及其对应的记录
q_traj_minus = flipud(q_traj_minus);
end_effector_pos_minus = flipud(end_effector_pos_minus);
mu_minus = fliplr(mu_minus);

% 合并正向和负向的关节角度轨迹
q_traj = [q_traj_minus; q_traj_plus];
end_effector_pos = [end_effector_pos_minus; end_effector_pos_plus];
mu = [mu_minus, mu_plus];

for i=1:length(q_traj)
    T1 = robot.fkine(q_traj(i,:));
    end_effector_orient(i, :) = tr2eul(T1);
end
% 计算末端点的位移长度
final_end_effector_pos_plus = end_effector_pos_plus(end, :);
final_end_effector_pos_minus = end_effector_pos_minus(end, :);
displacement_plus = norm(final_end_effector_pos_plus - initial_end_effector_pos);
displacement_minus = norm(final_end_effector_pos_minus - initial_end_effector_pos);
%% 

% 绘制关节角度随时间变化的曲线
figure;
plot(q_traj);
xlabel('Time step');
ylabel('Joint angles (rad)');
title('Joint Angles vs. Time');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
grid on;

% 绘制操控性随时间变化的曲线
figure;
plot(mu);
xlabel('Time step');
ylabel('Manipulability');
title('Manipulability vs. Time');
grid on;

% 绘制末端点坐标xyz随时间变化的曲线
figure;
plot3(end_effector_pos(:, 1), end_effector_pos(:, 2), end_effector_pos(:, 3));
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Position vs. Time');
grid on;


% 绘制末端姿态随时间变化的曲线（欧拉角）
figure;
plot(end_effector_orient);
xlabel('Time step');
ylabel('Euler angles (rad)');
title('End-Effector Orientation vs. Time');
legend('Roll', 'Pitch', 'Yaw');
grid on;


% 将旋转矩阵转换为欧拉角的函数
function eul = tr2eul(T1)
    % 提取旋转矩阵
    R = [T1.n, T1.o, T1.a];
    % 计算欧拉角
    sy = sqrt(R(1,1) * R(1,1) + R(2,1) * R(2,1));
    singular = sy < 1e-6;
    if ~singular
        x = atan2(R(3,2), R(3,3));
        y = atan2(-R(3,1), sy);
        z = atan2(R(2,1), R(1,1));
    else
        x = atan2(-R(2,3), R(2,2));
        y = atan2(-R(3,1), sy);
        z = 0;
    end
    eul = [x y z];
end









