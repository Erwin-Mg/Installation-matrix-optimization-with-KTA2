% function [mu_ba] = manu(robot, theta, constraint)
% %MANU 此处显示有关此函数的摘要
% %   此处显示详细说明
% switch constraint
%     case 0 %  位置+姿态约束
%         mu_ba = 0;
%         for i = 1:10000
%             if any(abs(theta) >= pi)
% %                 disp('A joint angle reached its limit.');
%                 break;
%             end
% %             J = geometricJacobian(robot, theta, 'end-effector');
%             J = robot.jacob0(theta);
%             J0 = J ; % 求解零空间6*n雅各比矩阵
%             null_space = null(J0); % 零空间向量
%             theta = theta + 0.001 * null_space' ;
% %             J_new = geometricJacobian(robot, theta, 'end-effector');
%             J_new = robot.jacob0(theta);
%             mu = sqrt(det(J_new*J_new')); %当前可操作度  
%             mu_ba = mu_ba + mu ;
%         end
%         mu_ba = mu_ba / 10000;
%     case 1 %  姿态约束
%         mu_ba = 0;
%         for i = 1:10000
%             if any(abs(theta) >= pi)
% %                 disp('A joint angle reached its limit.');
%                 break;
%             end
% %             J = geometricJacobian(robot, theta, 'end-effector');
%             J = robot.jacob0(theta);
%             J0 = J ; % 求解零空间6*n雅各比矩阵
%             null_space = null(J0); % 零空间向量
%             theta = theta + 0.001 * null_space ;
% %             J_new = geometricJacobian(robot, theta, 'end-effector');
%             J_new = robot.jacob0(theta);
%             mu = sqrt(det(J_new*J_new')); %当前可操作度  
%             mu_ba = mu_ba + mu ;
%         end
%         mu_ba = mu_ba / 10000;
% 
% 
% 
% end
% 






function [mu_ba] = manu(robot, theta, constraint)
%MANU 此处显示有关此函数的摘要
%   此处显示详细说明
num_steps = 10000;
mu_ba = 0;
theta = theta*pi/180;
switch constraint
    case 0 % 位置+姿态约束
        q_plus  = theta;
        q_minus = theta;
        mu_plus = zeros(1, num_steps); % 初始化 mu_plus
        mu_minus = zeros(1, num_steps); % 初始化 mu_minus

        % 可视化零空间运动并验证末端位置
        for i = 1:num_steps
            if any(abs(q_plus) >= pi) || any(abs(q_minus) >= pi)
%                 disp('A joint angle reached its limit.');
                break;
            end

            % 计算当前雅可比矩阵和伪逆
            J0_plus = robot.jacob0(q_plus);
            J0_minus = robot.jacob0(q_minus);

            % 计算零空间
            null_space_plus = null(J0_plus);
            null_space_minus = null(J0_minus);

            % 更新关节角度
            q_plus = q_plus + null_space_plus' * 0.001;
            q_minus = q_minus - null_space_minus' * 0.001;

            % 计算操控性
            mu_plus(i) = sqrt(det(J0_plus * J0_plus'));
            mu_minus(i) = sqrt(det(J0_minus * J0_minus'));
        end
        mu_ba = (mean(mu_plus) + mean(mu_minus)) / 2;

    case 1 % 姿态约束
        % 初始化关节角度
        q_plus = theta;
        q_minus = theta;
        mu_plus = zeros(1, num_steps); % 初始化 mu_plus
        mu_minus = zeros(1, num_steps); % 初始化 mu_minus

        % 可视化零空间运动并验证末端位置
        for i = 1:num_steps
            if any(abs(q_plus) >= pi) || any(abs(q_minus) >= pi)
%                 disp('A joint angle reached its limit.');
                break;
            end

            % 计算当前雅可比矩阵和伪逆
            J0_plus = robot.jacob0(q_plus);
            J0_minus = robot.jacob0(q_minus);

            % 计算零空间
            null_space_plus = null(J0_plus(1:3,:));
            null_space_minus = null(J0_minus(1:3,:));

            % 更新关节角度
            q_plus = q_plus + (null_space_plus * 0.001 * ones(4,1))';
            q_minus = q_minus - (null_space_minus * 0.001 * ones(4,1))';

            % 计算操控性
            mu_plus(i) = sqrt(det(J0_plus * J0_plus'));
            mu_minus(i) = sqrt(det(J0_minus * J0_minus'));
        end
        mu_ba = (mean(mu_plus) + mean(mu_minus)) / 2;
end
end



