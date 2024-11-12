function [D1,Length,max_seq_length,max_length] = compute_length(D,axis)
% 示例 D 矩阵，假设前三列为xyz坐标，后两列为其他参数
% D = [
%     1 2 3 0.1 0.2;
%     1 1 3 0.1 0.2;
%     1 3 3 0.1 0.2;
%     1 4 3 0.1 0.2;
%     1 5 3 0.1 0.2;
%     2 3 1 0.2 0.3;
%     2 1 3 0.3 0.4;
%     2 2 3 0.3 0.4;
%     2 3 3 0.3 0.4;
%     2 4 3 0.3 0.4;
%     3 2 3 0.1 0.2;
%     3 1 3 0.1 0.2;
%     3 4 3 0.1 0.2;
%     3 3 3 0.1 0.2;
%     3 6 3 0.1 0.2;
%     3 7 3 0.1 0.2;
% ];
switch axis
    case 1
        % 获取所有唯一的第 2、3、4、5 列组合. 计算x方向上最长序列
        % 筛选第4列元素属于 [-29.5, -27.5] 或 [27.5, 29.5] 的行
        cond4 = (D(:, 4) >= -29.5 & D(:, 4) <= -27.5) | (D(:, 4) >= 27.5 & D(:, 4) <= 29.5);
        % 筛选第5列元素属于 [-1.5, 1.5] 的行
        cond5 = D(:, 5) >= -1.5 & D(:, 5) <= 1.5;
        % 结合两个条件筛选行
        D = D(cond4 & cond5, :);
        D1 = sortrows(D, [4 5 2 3 1]);
        [unique_combinations, ~, group_indices] = unique(D1(:, 2:5), 'rows');
    case 2
        % 获取所有唯一的第 1、3、4、5 列组合
        % 筛选第4列元素属于 [-12.5, -17.5]的行
        cond4 = (D(:, 4) >= -17.5 & D(:, 4) <= -12.5);
        % 筛选第5列元素属于 [-1.5, 1.5] 的行
        cond5 = D(:, 5) >= -1.5 & D(:, 5) <= 1.5;
        % 结合两个条件筛选行
        D = D(cond4 & cond5, :);        
        D1 = sortrows(D, [4 5 1 3 2]);
        [unique_combinations, ~, group_indices] = unique(D1(:, [1 3 4 5]), 'rows');
    case 3
        % 获取所有唯一的第 1、2、4、5 列组合
        % 筛选位于Z轴负方向的点
        cond5 = D(:, 5) == -14.5 ;
        % 结合两个条件筛选行
        D = D( cond5, :);
        D1 = sortrows(D, [4 5 1 2 3]);
        [unique_combinations, ~, group_indices] = unique(D1(:, [1 2 4 5]), 'rows');
end
% 初始化结果存储
num_groups = size(unique_combinations, 1);
% result = cell(num_groups, 1);

% 变量来存储具有最大序列长度的分组信息
max_length = 0;
% max_indices = [];
% max_combination = [];
length_num = 0;         %
% 遍历每个唯一组合
max_seq_length = 0;

%%
for i = 1:num_groups
    % 获取当前组合对应的行索引
    current_group_indices = find(group_indices == i);
    % 获取当前分组的第axis列元素
    col_elements = D1(current_group_indices, axis);
    % 检查第二列是否存在连续递增序列
    current_seq_length = 1;
%     seq_start_index = current_group_indices(1);
    ff = false;
    if length(col_elements) > 1
        for j = 2:length(col_elements)
            if col_elements(j) - col_elements(j-1) == 1
                current_seq_length = current_seq_length + 1;
                ff = true;
                if j == length(col_elements)   %连续序列至该组最后一项
                    length_num = length_num + 1;                                    % 连续序列个数 length_num + 1
                    Length{length_num}.indice = [j-current_seq_length+1   j]+current_group_indices(1)-1;         % 连续序列起止行数
                    Length{length_num}.length = current_seq_length ;                % 连续序列长度
                    if current_seq_length >max_seq_length           % 计算最长序列长度
                        max_seq_length = current_seq_length;        %长度
                        max_length = Length{length_num}.indice ;    %起止行数
                    end
                end
            elseif ff  % 连续序列未至最后一项且连续序列中止
                length_num = length_num + 1;                                    % 连续序列个数 length_num + 1
                Length{length_num}.indice = [j-current_seq_length j-1]+current_group_indices(1)-1;         % 连续序列起止行数
                Length{length_num}.length = current_seq_length ;                % 连续序列长度
                if current_seq_length >max_seq_length           % 计算最长序列长度
                    max_seq_length = current_seq_length;        %长度
                    max_length = Length{length_num}.indice ;    %起止行数
                end
                ff = false;
                current_seq_length = 1;  % 将该组序列长度重置为1以开启新连续序列计算
            end
        end
    end
end



end