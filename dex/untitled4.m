% 示例 D 矩阵，假设前三列为xyz坐标，后两列为其他参数
D = [
    1 2 3 0.1 0.2;
    1 1 3 0.1 0.2;
    1 3 3 0.1 0.2;
    1 4 3 0.1 0.2;
    1 5 3 0.1 0.2;
    1 3 3 0.1 0.2;
    2 3 1 0.2 0.3;
    2 1 3 0.3 0.4;
    2 2 3 0.3 0.4;
    2 3 3 0.3 0.4;
    2 4 3 0.3 0.4;
    2 4 3 0.3 0.4;
    3 2 3 0.1 0.2;
    3 3 3 0.1 0.2;
    3 4 3 0.1 0.2;
    3 5 3 0.1 0.2;
    3 7 3 0.1 0.2;
    3 8 3 0.1 0.2;
];

% 获取所有唯一的第 1、3、4、5 列组合
[unique_combinations, ~, group_indices] = unique(D(:, [1 3 4 5]), 'rows');

% 初始化结果存储
num_groups = size(unique_combinations, 1);
result = cell(num_groups, 1);

% 变量来存储具有最大序列长度的分组信息
max_length = 0;
max_indices = [];
max_combination = [];

% 遍历每个唯一组合
for i = 1:num_groups
    % 获取当前组合对应的行索引
    current_group_indices = find(group_indices == i);
    % 获取当前分组的第二列元素
    second_col_elements = D(current_group_indices, 2);
    % 检查第二列是否存在连续递增序列
    max_seq_length = 0;
    current_seq_length = 1;
    seq_start_index = current_group_indices(1);
    for j = 1:length(second_col_elements) - 1
        if second_col_elements(j + 1) - second_col_elements(j) == 1
            current_seq_length = current_seq_length + 1;
        else
            if current_seq_length > max_seq_length
                max_seq_length = current_seq_length;
                seq_start_index = current_group_indices(j - current_seq_length + 1);
            end
            current_seq_length = 1;
        end
    end
    if current_seq_length > max_seq_length
        max_seq_length = current_seq_length;
        seq_start_index = current_group_indices(end - current_seq_length + 1);
    end
    % 存储结果
    result{i} = struct('combination', unique_combinations(i, :), 'max_seq_length', max_seq_length, ...
        'indices', current_group_indices(seq_start_index - current_group_indices(1) + 1 : seq_start_index - current_group_indices(1) + max_seq_length));
    
    % 更新具有最大连续序列长度的分组信息
    if max_seq_length > max_length
        max_length = max_seq_length;
        max_indices = current_group_indices(seq_start_index - current_group_indices(1) + 1 : seq_start_index - current_group_indices(1) + max_seq_length);
        max_combination = unique_combinations(i, :);
    end
end

% % 显示具有最大连续序列长度的分组信息
% disp('Unique Combination with Maximum Sequence:');
% disp(max_combination);
% disp('Maximum Sequence Length in the Second Column:');
% disp(max_length);
% disp('Indices of the rows with this combination:');
% disp(max_indices);
% disp('Start and End Indices of the Maximum Sequence:');
% disp(['Start Index: ', num2str(max_indices(1))]);
% disp(['End Index: ', num2str(max_indices(end))]);
j=1;
for i=1:num_groups
    if result{i}.max_seq_length>=2
        indice(j,1:2) = [result{i}.indices(1),result{i}.indices(end)];
        j=j+1;
    end
end