function F = CalObj(point_coordinate,robotArm,R_BA1,Jacob,endlocation,W,MF)
for t=1:size(point_coordinate,1)
% global M D name
% 确定机械臂参数
% 定义D-H参数
% function f = CalObj(point_coordinate,robotArm,R_BA1,Jacob,endlocation,N,MF)

f=[];
l=1;
% robotArm=robotArm_initial;
% endlocation=endlocation_1;
% N=10000;
% 
% 设立 关节刚度K_q
K_q=[2.8e4 2.1e4 2.8e4 7.1e3 6.9e3 2.8e4 5.2e3];
% 创建四个单臂机器人
robotArm1 = robotArm{1};
robotArm2 = robotArm{2};
robotArm3 = robotArm{3};
robotArm4 = robotArm{4};
j=7;


% 确定四臂根部位置
vertex_A = point_coordinate(t,1:3);
vertex_B = point_coordinate(t,4:6);
vertex_C = point_coordinate(t,7:9);
vertex_D = point_coordinate(t,10:12);

x = [ vertex_A(1),vertex_B(1),vertex_C(1),vertex_D(1)];
y = [ vertex_A(2),vertex_B(2),vertex_C(2),vertex_D(2)];
z = [ vertex_A(3),vertex_B(3),vertex_C(3),vertex_D(3)];

% 计算坐标转换矩阵
line_BA = vertex_B - vertex_A;
line_CA = vertex_C - vertex_A;
line_DA = vertex_D - vertex_A;
% 单位化连线向量
vector_BA = line_BA / norm(line_BA);
vector_CA = line_CA / norm(line_CA);
vector_DA = line_DA / norm(line_DA);


n_BA=cross(vector_BA,[0,0,1])';
o_BA=cross(n_BA,vector_BA)';
R_BA=tr2eul([n_BA,o_BA,vector_BA']);
trans_matixBA = rotz(R_BA(1)*180/pi)*roty(R_BA(2)*180/pi)*rotz(R_BA(3)*180/pi);

n_CA=cross(vector_CA,[0,0,1])';
o_CA=cross(n_CA,vector_CA)';
R_CA=tr2eul([n_CA,o_CA,vector_CA']);
trans_matixCA = rotz(R_CA(1)*180/pi)*roty(R_CA(2)*180/pi)*rotz(R_CA(3)*180/pi);

n_DA=cross(vector_DA,[0,0,1])';
o_DA=cross(n_DA,vector_DA)';
R_DA=tr2eul([n_DA,o_DA,vector_DA']);
trans_matixDA = rotz(R_DA(1)*180/pi)*roty(R_DA(2)*180/pi)*rotz(R_DA(3)*180/pi);

% 为每个机器人臂定义基坐标系相对于正四面体底面顶点的变换矩阵
T_arm1 = transl(vertex_A);
T_arm2 = transl(vertex_B) * trotz(R_BA(1)*180/pi)*troty(R_BA(2)*180/pi)*trotz(R_BA(3)*180/pi);
T_arm3 = transl(vertex_C) * trotz(R_CA(1)*180/pi)*troty(R_CA(2)*180/pi)*trotz(R_CA(3)*180/pi);
T_arm4 = transl(vertex_D) * trotz(R_DA(1)*180/pi)*troty(R_DA(2)*180/pi)*trotz(R_DA(3)*180/pi);

% 为每个机器人臂应用变换矩阵

% four_arm_robot.arm1.base = T_arm1;
% four_arm_robot.arm2.base = T_arm2;
% four_arm_robot.arm3.base = T_arm3;
% four_arm_robot.arm4.base = T_arm4;

robotArm1.base = T_arm1;
robotArm2.base = T_arm2;
robotArm3.base = T_arm3;
robotArm4.base = T_arm4;

%% 刚度计算
% % endlocation_1=endlocation_1
% endlocation122=endloacation;
[~,a] = size(endlocation);
endlocation_1 = zeros(W,a);
endlocation_1(:,1:3)=endlocation(:,1:3)+vertex_A;
endlocation_1(:,4:10)=endlocation(:,4:end);
% 获得剩余三臂工作空间
Q_2=zeros(W,j+3);
Q_3=zeros(W,j+3);
Q_4=zeros(W,j+3);

% 创建偏移矩阵
offset_B = [x(2)-x(1); y(2)-y(1); z(2)-z(1)];
offset_C = [x(3)-x(1); y(3)-y(1); z(3)-z(1)];
offset_D = [x(4)-x(1); y(4)-y(1); z(4)-z(1)];

% 将 endlocation_1 的位置信息和关节角信息分开
endlocation_1(:, 1:3)=endlocation_1(:, 1:3)+vertex_A;
positions = endlocation_1(:, 1:3);
joint_angles = endlocation_1(:, 4:end);

Q_2(:, 1:3) = (trans_matixBA * (positions' + offset_B - vertex_B') + vertex_B')';
Q_3(:, 1:3) = (trans_matixCA * (positions' + offset_C - vertex_C') + vertex_C')';
Q_4(:, 1:3) = (trans_matixDA * (positions' + offset_D - vertex_D') + vertex_D')';

% 复制关节角信息
Q_2(:, 4:j+3) = joint_angles;
Q_3(:, 4:j+3) = joint_angles;
Q_4(:, 4:j+3) = joint_angles;

points=[vertex_A;vertex_B;vertex_C;vertex_D];

alpha_shape=alphaShape(points);
if volume(alpha_shape)==0
    zz=[0 0 0.2];
    points=[vertex_A+zz;vertex_B+zz;vertex_C+zz;vertex_D+zz;
        vertex_A-zz;vertex_B-zz;vertex_C-zz;vertex_D-zz];
    alpha_shape=alphaShape(points);
end
% 粗略筛除与基座干涉末端点，未考虑自避障
in_alpha_shape = inShape(alpha_shape, endlocation_1(:,1), endlocation_1(:,2), endlocation_1(:,3));
endlocation_1=endlocation_1(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_2(:,1), Q_2(:,2), Q_2(:,3));
Q_2=Q_2(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_3(:,1), Q_3(:,2), Q_3(:,3));
Q_3=Q_3(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_4(:,1), Q_4(:,2), Q_4(:,3));
Q_4=Q_4(~in_alpha_shape, :);
clear in_alpha_shape


%计算各个根部之间的方向向量
vector_12=vertex_A-vertex_B;
vector_13=vertex_A-vertex_C;
vector_14=vertex_A-vertex_D;
vector_23=vertex_B-vertex_C;
vector_24=vertex_B-vertex_D;
vector_34=vertex_C-vertex_D;

pos_switchBA=l*(vector_12 / norm(vector_12));
pos_switchCA=l*(vector_13 / norm(vector_13));
pos_switchDA=l*(vector_14 / norm(vector_14));
pos_switchCB=l*(vector_23 / norm(vector_23));
pos_switchDB=l*(vector_24 / norm(vector_24));
pos_switchDC=l*(vector_34 / norm(vector_34));

pos_switchBA_mat = repmat(pos_switchBA, length(Q_2), 1);
pos_switchCA_mat = repmat(pos_switchCA, length(Q_3), 1);
pos_switchDA_mat = repmat(pos_switchDA, length(Q_4), 1);
pos_switchCB_mat = repmat(pos_switchCB, length(Q_3), 1);
pos_switchDB_mat = repmat(pos_switchDB, length(Q_4), 1);
pos_switchDC_mat = repmat(pos_switchDC, length(Q_4), 1);

% 平移末端点
Q_21trans = Q_2(:, 1:3) + pos_switchBA_mat;
Q_31trans = Q_3(:, 1:3) + pos_switchCA_mat;
Q_41trans = Q_4(:, 1:3) + pos_switchDA_mat;
Q_32trans = Q_3(:, 1:3) + pos_switchCB_mat;
Q_42trans = Q_4(:, 1:3) + pos_switchDB_mat;
Q_43trans = Q_4(:, 1:3) + pos_switchDC_mat;
% 筛除夹持物块后与基座干涉点
in_alpha_shape = inShape(alpha_shape, Q_21trans(:,1), Q_21trans(:,2), Q_21trans(:,3));
Q_21trans= Q_21trans(~in_alpha_shape, :);
clear in_alpha_shape
Q_31trans=shaixuan(Q_31trans,alpha_shape);
Q_41trans=shaixuan(Q_41trans,alpha_shape);
Q_32trans=shaixuan(Q_32trans,alpha_shape);
Q_42trans=shaixuan(Q_42trans,alpha_shape);
Q_43trans=shaixuan(Q_43trans,alpha_shape);

points_rete={endlocation_1(:,1:3) , Q_2(:,1:3), Q_3(:,1:3) , Q_4(:,1:3) };

% alpha_rete1 = alphaShape(endlocation_1(:,1:3));
% alpha_rete2 = alphaShape(Q_2(:,1:3));
% alpha_rete3 = alphaShape(Q_3(:,1:3));
% alpha_rete4 = alphaShape(Q_4(:,1:3));
alpha_tans21= alphaShape(Q_21trans);
alpha_tans31= alphaShape(Q_31trans);
alpha_tans41= alphaShape(Q_41trans);
alpha_tans32= alphaShape(Q_32trans);
alpha_tans42= alphaShape(Q_42trans);
alpha_tans43= alphaShape(Q_43trans);



% alpha_rete = { alpha_rete1, alpha_rete2, alpha_rete3, alpha_rete4 };
alpha_trans= { alpha_tans21, alpha_tans31, alpha_tans41, alpha_tans32,...
                                            alpha_tans42, alpha_tans43 };


% points_cross=points_rete{2}(inShape(alpha_tans32, points_rete{2}), :);
k=[3 2 1];
p=0;
w=0;
points_cross=cell(1,6);
for i=1:3
    for j=1:k(i)
        points_cross{j+w}=points_rete{i}(inShape((alpha_trans{j+w}),points_rete{i}),:);
        p=p+1;
    end
    w=p;
end



% figure ;
% plot(alpha_rete1, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
% hold on;
% plot(alpha_rete2, 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
% plot(alpha_rete3, 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
% plot(alpha_rete4, 'FaceColor', 'black', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);

% 得到每两个臂的交集
coop12=points_cross{1};
coop13=points_cross{2};
coop14=points_cross{3};
coop23=points_cross{4};
coop24=points_cross{5};
coop34=points_cross{6};

coop12recover=coop12-pos_switchBA;
coop13recover=coop13-pos_switchCA;
coop14recover=coop14-pos_switchDA;
coop23recover=coop23-pos_switchCB;
coop24recover=coop24-pos_switchDB;
coop34recover=coop34-pos_switchDC;


points_recover={coop12recover, coop13recover, coop14recover, coop23recover, coop24recover, coop34recover};
% 若四条臂均无交集则跳出循环
if isempty(points_recover)
        F=[100 100 100 100 100];
    return;
else
% alpha_coop12recover=alphaShape(coop12recover);
alpha_coop12=alphaShape(coop12);
alpha_coop13=alphaShape(coop13);
alpha_coop14=alphaShape(coop14);
alpha_coop23=alphaShape(coop23);
alpha_coop24=alphaShape(coop24);
alpha_coop34=alphaShape(coop34);
alpha_coop = {alpha_coop12,alpha_coop13,alpha_coop14,alpha_coop23,alpha_coop24,alpha_coop34};


%% 计算刚度指标

% 运动学逆解并求出雅各比矩阵

J12_2=cell(1,myLength(coop12));
MF12_2=cell(1,myLength(coop12));


J13_3=cell(1,myLength(coop13));
MF13_3=cell(1,myLength(coop13));

J14_4=cell(1,myLength(coop14));
MF14_4=cell(1,myLength(coop14));

J23_3=cell(1,myLength(coop23));
MF23_3=cell(1,myLength(coop23));

J24_4=cell(1,myLength(coop24));
MF24_4=cell(1,myLength(coop24));

J34_4=cell(1,myLength(coop34));
MF34_4=cell(1,myLength(coop34));
%% 

%臂1与臂2
clear matching_rows
matching_rows = ismember(endlocation_1(:, 1:3), coop12, 'rows');
% 找到匹配的行的索引
matching_row_indices12 = find(matching_rows);
% 提取对应的数据
% angel12_1 = endlocation_1(matching_row_indices12, 4:end);  
%正方体筛选边长
L = 1;
II =round(W/4230*L^3) ;%小正方体内点数
J12_1 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices12), 'UniformOutput', false);
MF12_1= cellfun(@(index) MF{index}, num2cell(matching_row_indices12), 'UniformOutput', false);

parfor i=1:myLength(coop12recover)
    point_A = coop12recover(i,:);
    % 筛选出在正方体内的点
    inside_cube_mask = all(Q_2(:,1:3) >= point_A - L/2 & Q_2(:,1:3) <= point_A + L/2, 2);    
    % 找到在正方体内的点的索引
    if nnz(inside_cube_mask)<=II
        J12_2{i}=0;
    else
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_2(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J12_2{i}=Jacob{idx}; 
        MF12_2{i}=MF{idx};
    end
end
%% 

% % 臂1与臂3
clear matching_rows 
matching_rows = ismember(endlocation_1(:, 1:3), coop13, 'rows');
% 找到匹配的行的索引
matching_row_indices13 = find(matching_rows);
% 提取对应的数据
% angel13_1 = endlocation_1(matching_row_indices13, 4:end);
J13_1 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices13), 'UniformOutput', false);
MF13_1= cellfun(@(index) MF{index}, num2cell(matching_row_indices13), 'UniformOutput', false);

parfor i=1:myLength(coop13recover)
    point_A = coop13recover(i,:);
    inside_cube_mask = all(Q_3(:,1:3) >= point_A - L/2 & Q_3(:,1:3) <= point_A + L/2, 2);    
    if nnz(inside_cube_mask)<=II
        J13_3{i}=0;
    else
        % 找到在正方体内的点的索引
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_3(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J13_3{i}=Jacob{idx}; 
        MF13_3{i}=MF{idx};
    end
end
%% 

%臂1与臂4
clear matching_rows 
matching_rows = ismember(endlocation_1(:, 1:3), coop14, 'rows');
% 找到匹配的行的索引
matching_row_indices14 = find(matching_rows);
% 提取对应的数据
% angel14_1 = endlocation_1(matching_row_indices14, 4:end);
J14_1 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices14), 'UniformOutput', false);
MF14_1= cellfun(@(index) MF{index}, num2cell(matching_row_indices14), 'UniformOutput', false);

parfor i=1:myLength(coop14recover)
    point_A = coop14recover(i,:);
    inside_cube_mask = all(Q_4(:,1:3) >= point_A - L/2 & Q_4(:,1:3) <= point_A + L/2, 2); 
    if nnz(inside_cube_mask)<=II
        J14_4{i}=0;
    else
        % 找到在正方体内的点的索引
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_4(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J14_4{i}=Jacob{idx};
        MF14_4{i}=MF{idx};
    end
end
%% 

%臂2与臂3
clear matching_rows 
matching_rows = ismember(Q_2(:, 1:3), coop23, 'rows');
% 找到匹配的行的索引
matching_row_indices23 = find(matching_rows);
% 提取对应的数据
% angel23_2 = Q_2(matching_row_indices23, 4:end);
J23_2 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices23), 'UniformOutput', false);
MF23_2= cellfun(@(index) MF{index}, num2cell(matching_row_indices23), 'UniformOutput', false);

parfor i=1:myLength(coop23recover)
    point_A = coop23recover(i,:);
    inside_cube_mask = all(Q_3(:,1:3) >= point_A - L/2 & Q_3(:,1:3) <= point_A + L/2, 2);    
    if nnz(inside_cube_mask)<=II
        J23_3{i}=0;
    else
        % 找到在正方体内的点的索引
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_3(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J23_3{i}=Jacob{idx}; 
        MF23_3{i}=MF{idx};
    end
end
%% 

%臂2与臂4

clear matching_rows 
matching_rows = ismember(Q_2(:, 1:3), coop24, 'rows');
% 找到匹配的行的索引
matching_row_indices24 = find(matching_rows);
% 提取对应的数据
% angel24_2 = Q_2(matching_row_indices24, 4:end);
J24_2 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices24), 'UniformOutput', false);
MF24_2= cellfun(@(index) MF{index}, num2cell(matching_row_indices24), 'UniformOutput', false);

parfor i=1:myLength(coop24recover)
    point_A = coop24recover(i,:);
    inside_cube_mask = all(Q_4(:,1:3) >= point_A - L/2 & Q_4(:,1:3) <= point_A + L/2, 2);    
    if nnz(inside_cube_mask)<=II
        J24_4{i}=0;
    else
        % 找到在正方体内的点的索引
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_4(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J24_4{i}=Jacob{idx}; 
        MF24_4{i}=MF{idx};
    end
end
%% 

%臂3与臂4
clear matching_rows 
matching_rows = ismember(Q_3(:, 1:3), coop34, 'rows');
% 找到匹配的行的索引
matching_row_indices34 = find(matching_rows);
% 提取对应的数据
% angel34_3= Q_3(matching_row_indices34, 4:end);
J34_3 = cellfun(@(index) Jacob{index}, num2cell(matching_row_indices34), 'UniformOutput', false);
MF34_3= cellfun(@(index) MF{index}, num2cell(matching_row_indices34), 'UniformOutput', false);

parfor i=1:myLength(coop34recover)
    point_A = coop34recover(i,:);
    inside_cube_mask = all(Q_4(:,1:3) >= point_A - L/2 & Q_4(:,1:3) <= point_A + L/2, 2);    
    if nnz(inside_cube_mask)<=II
        J34_4{i}=0;
    else
        % 找到在正方体内的点的索引
        indices_inside_cube = inside_cube_mask;
        % 提取在正方体内的点
        points_inside_cube = Q_4(indices_inside_cube, 1:3);
        distances = sqrt(sum((points_inside_cube - point_A).^2, 2));    
        % 找到距离最小的点的索引
        [~, idx] = min(distances);
        J34_4{i}=Jacob{idx}; 
        MF34_4{i}=MF{idx};
    end
end
%% 计算刚度矩阵
% points = [vertex_A; vertex_B; vertex_C; vertex_D];
% % 将A在两杆连线中点的新坐标系下表示

points_root={vertex_A,vertex_B,vertex_C,vertex_D};
L=1;
O=cell(1,6);
midpoints=zeros(6,3);
coordinate_system=cell(1,6);
for i=1:4
    for j=i+1:4
        O{L}=(points_root{i}+points_root{j})/2;
        midpoints(L,1:3)=O{L}(1:3);
        P=points_root{i}-points_root{j};
        X_axis=P/norm(P);
        Y_axis=cross(X_axis, [0, 0, 1]);
        Y_axis = Y_axis / norm(Y_axis);

        Z_axis = cross(X_axis, Y_axis);
        Z_axis = Z_axis / norm(Z_axis);
        
        coordinate_system{L}=[X_axis', Y_axis', Z_axis'];
        L=L+1;
    end
end

% % 计算刚度矩阵
% % 臂 1 & 臂 2  

K12=cell(1,myLength(coop12));
if ~isempty(coop12)
    parfor i=1:myLength(coop12)
        if length(J12_2{i})==7
            P_c12_1 = skew((midpoints(1,:) + (coop12(i,:)-coop12recover(i,:)))/2);
            P_c12_2 = skew((midpoints(1,:) + (coop12recover(i,:)-coop12(i,:)))/2);
            K121=[eye(3) zeros(3,3);-P_c12_1 eye(3)];
            K122=[eye(3) zeros(3,3);-P_c12_2 eye(3)];
            K12_1=K121*(pinv(J12_1{i}))'*diag(K_q)*pinv(J12_1{i})*K121';
            K12_2=K122*(pinv(J12_2{i}))'*diag(K_q)*pinv(J12_2{i})*K122';
            K12{i}=K12_1+K12_2;
        else
            K12{i}=0;
        end
    end
else 
    K12=cell(1,0);
end

% 臂 1 & 臂 3
K13=cell(1,myLength(coop13));
if ~isempty(coop13)
    parfor i=1:myLength(coop13)
        if length(J13_3{i})==7
            P_c13_1 = skew((midpoints(2,:) + (coop13(i,:)-coop13recover(i,:)))/2);
            P_c13_3 = skew((midpoints(2,:) + (coop13recover(i,:)-coop13(i,:)))/2);
            K131=[eye(3) zeros(3,3);-P_c13_1 eye(3)];
            K133=[eye(3) zeros(3,3);-P_c13_3 eye(3)];
            K13_1=K131*(pinv(J13_1{i}))'*diag(K_q)*pinv(J13_1{i})*K131';
            K13_3=K133*(pinv(J13_3{i}))'*diag(K_q)*pinv(J13_3{i})*K133';
            K13{i}=K13_1+K13_3;
        else
            K13{i}=0;
        end
    end
else 
    K13=cell(1,0);
end

% 臂 1 & 臂 4
K14=cell(1,myLength(coop14));
if ~isempty(coop14)
    parfor i=1:myLength(coop14)
        if length(J14_4{i})==7
            P_c14_1 = skew((midpoints(3,:) + (coop14(i,:)-coop14recover(i,:)))/2);
            P_c14_4 = skew((midpoints(3,:) + (coop14recover(i,:)-coop14(i,:)))/2);
            K141=[eye(3) zeros(3,3);-P_c14_1 eye(3)];
            K144=[eye(3) zeros(3,3);-P_c14_4 eye(3)];
            K14_1=K141*(pinv(J14_1{i}))'*diag(K_q)*pinv(J14_1{i})*K141';
            K14_4=K144*(pinv(J14_4{i}))'*diag(K_q)*pinv(J14_4{i})*K144';
            K14{i}=K14_1+K14_4;
        else
            K14{i}=0;
        end
    end
else
    K14=cell(1,0);
end

% 臂 2 & 臂 3
K23=cell(1,myLength(coop23));
if ~isempty(coop23)
    parfor i=1:myLength(coop23)
        if length(J23_3{i})==7
            P_c23_2 = skew((midpoints(4,:) + (coop23(i,:)-coop23recover(i,:)))/2);
            P_c23_3 = skew((midpoints(4,:) + (coop23recover(i,:)-coop23(i,:)))/2);
            K232=[eye(3) zeros(3,3);-P_c23_2 eye(3)];
            K233=[eye(3) zeros(3,3);-P_c23_3 eye(3)];
            K23_2=K232*(pinv(J23_2{i}))'*diag(K_q)*pinv(J23_2{i})*K232';
            K23_3=K233*(pinv(J23_3{i}))'*diag(K_q)*pinv(J23_3{i})*K233';
            K23{i}=K23_2+K23_3;
        else
            K23{i}=0;
        end
    end
else 
    K23=cell(1,0);
end


% 臂 2 & 臂 4
K24=cell(1,myLength(coop24));
if ~isempty(coop24)
    parfor i=1:myLength(coop24)
        if length(J24_4{i})==7
            P_c24_2 = skew((midpoints(5,:) + (coop24(i,:)-coop24recover(i,:)))/2);
            P_c24_4 = skew((midpoints(5,:) + (coop24recover(i,:)-coop24(i,:)))/2);
            K242=[eye(3) zeros(3,3);-P_c24_2 eye(3)];
            K244=[eye(3) zeros(3,3);-P_c24_4 eye(3)];
            K24_2=K242*(pinv(J24_2{i}))'*diag(K_q)*pinv(J24_2{i})*K242';
            K24_4=K244*(pinv(J24_4{i}))'*diag(K_q)*pinv(J24_4{i})*K244';
            K24{i}=K24_2+K24_4;
        else
            K24{i}=0;
        end
    end
else 
    K24=cell(1,0);
end

% 臂 3 & 臂 4
K34=cell(1,myLength(coop34));
if ~isempty(coop34)
    parfor i=1:myLength(coop34)
        if length(J34_4{i})==7
            P_c34_3 = skew((midpoints(6,:) + (coop34(i,:)-coop34recover(i,:)))/2);
            P_c34_4 = skew((midpoints(6,:) + (coop34recover(i,:)-coop34(i,:)))/2);
            K343=[eye(3) zeros(3,3);-P_c34_3 eye(3)];
            K344=[eye(3) zeros(3,3);-P_c34_4 eye(3)];
            K34_3=K343*(pinv(J34_3{i}))'*diag(K_q)*pinv(J34_3{i})*K343';
            K34_4=K344*(pinv(J34_4{i}))'*diag(K_q)*pinv(J34_4{i})*K344';
            K34{i}=K34_3+K34_4;
        else
            K34{i}=0;
        end
    end
else 
    K34=cell(1,0);
end

K=[K12,K13,K14,K23,K24,K34];
j=1;
min_singular_value12=[];
min_singular_value13=[];
min_singular_value14=[];
min_singular_value23=[];
min_singular_value24=[];
min_singular_value34=[];
if isempty(K)
    f(3)=100;
else
    for i=1:length(K12)
        if length(K12{i})==6
            [~, S, ~] = svd(K12{i});
            singular_values = diag(S);
            min_singular_value12(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value12)
        min12=0;
    else
        min12=min(min_singular_value12);
    end

    for i=1:length(K13)
        if length(K13{i})==6
            [~, S, ~] = svd(K13{i});
            singular_values = diag(S);
            min_singular_value13(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value13)
        min13=0;
    else
        min13=min(min_singular_value13);
    end
    for i=1:length(K14)
        if length(K14{i})==6
            [~, S, ~] = svd(K14{i});
            singular_values = diag(S);
            min_singular_value14(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value14)
        min14=0;
    else
        min14=min(min_singular_value14);
    end

    for i=1:length(K23)
        if length(K23{i})==6
            [~, S, ~] = svd(K23{i});
            singular_values = diag(S);
            min_singular_value23(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value23)
        min23=0;
    else
        min23=min(min_singular_value23);
    end 

    for i=1:length(K24)
        if length(K24{i})==6
            [~, S, ~] = svd(K24{i});
            singular_values = diag(S);
            min_singular_value24(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value24)
        min24=0;
    else
        min24=min(min_singular_value24);
    end

    for i=1:length(K34)
        if length(K34{i})==6
            [~, S, ~] = svd(K34{i});
            singular_values = diag(S);
            min_singular_value34(i) = min(singular_values);
        else
            continue;
        end
    end
    if isempty(min_singular_value34)
        min34=0;
    else
        min34=min(min_singular_value34);
    end 
    min_K=[min34,min24,min23,min14,min13,min12];
    min_f3=[min34+min12,min24+min13,min23+min14];
    % % 获取最小奇异值f3并返回
    f(3) = -max(min_f3);
    if f(3)==0
        f(3)=100;
    end
end
%获得双臂操作最小奇异值
% K_min=min(min_singular_value);
% 提取奇异值的向量
% singular_values = diag(S);

% % 获取最小奇异值
% singular_value = min(singular_values);

%% 计算运动学灵巧性-相对雅各比矩阵
base1=[robotArm1.base.n , robotArm1.base.o , robotArm1.base.a ];
base2=[robotArm2.base.n , robotArm2.base.o , robotArm2.base.a ];
base3=[robotArm3.base.n , robotArm3.base.o , robotArm3.base.a ];
base4=[robotArm4.base.n , robotArm4.base.o , robotArm4.base.a ];

% 臂1与臂2
relative_jacob12=cell(1,myLength(coop12));
MM_relative_jacob12 = zeros(1,myLength(coop12));
if ~isempty(coop12)
    parfor i=1:myLength(coop12)
        if length(J12_2{i})==7
            %计算两末端点向量p
            p = coop12recover(i,:)-coop12(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices12(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J12_1{i};
            JB = J12_2{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base1'*base2;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob12{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob12(i) = sqrt(det(relative_jacob12{i}*relative_jacob12{i}'));
        else
            MM_relative_jacob12(i)=0;
        end
    end
    MM_relative_jacob(1)=max(MM_relative_jacob12);
else
    MM_relative_jacob(1)=0;
end



% 臂1与臂3
relative_jacob13=cell(1,myLength(coop13));
MM_relative_jacob13 = zeros(1,myLength(coop13));
if ~isempty(coop13)
    parfor i=1:myLength(coop13)
        if length(J13_3{i})==7
            %计算两末端点向量p
            p = coop13recover(i,:)-coop13(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices13(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J13_1{i};
            JB = J13_3{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base1'*base3;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob13{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob13(i) = sqrt(det(relative_jacob13{i}*relative_jacob13{i}'));
        else
            MM_relative_jacob13(i)=0;
        end
    end
    MM_relative_jacob(2)=max(MM_relative_jacob13);
else
    MM_relative_jacob(2)=0;
end
% 臂1与臂4
relative_jacob14=cell(1,myLength(coop14));
MM_relative_jacob14 = zeros(1,myLength(coop14));
if ~isempty(coop14)
    parfor i=1:myLength(coop14)
        if length(J14_4{i})==7
            %计算两末端点向量p
            p = coop14recover(i,:)-coop14(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices14(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J14_1{i};
            JB = J14_4{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base1'*base4;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob14{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob14(i) = sqrt(det(relative_jacob14{i}*relative_jacob14{i}'));
        else
            MM_relative_jacob14(i)=0;
        end
    end
    MM_relative_jacob(3)=max(MM_relative_jacob14);
else 
    MM_relative_jacob(3)=0;
end

% 臂2与臂3
relative_jacob23=cell(1,myLength(coop23));
MM_relative_jacob23 = zeros(1,myLength(coop23));
if ~isempty(coop23)
    parfor i=1:myLength(coop23)
        if length(J23_3{i})==7
            %计算两末端点向量p
            p = coop23recover(i,:)-coop23(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices23(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J23_2{i};
            JB = J23_3{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base2'*base3;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob23{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob23(i) = sqrt(det(relative_jacob23{i}*relative_jacob23{i}'));
        else
            MM_relative_jacob23(i)=0;
        end
    end
    MM_relative_jacob(4)=max(MM_relative_jacob23);
else
    MM_relative_jacob(4)=0;
end

% 臂2与臂4
relative_jacob24=cell(1,myLength(coop24));
MM_relative_jacob24 = zeros(1,myLength(coop24));
if ~isempty(coop24)
    parfor i=1:myLength(coop24)
        if length(J24_4{i})==7
            %计算两末端点向量p
            p = coop24recover(i,:)-coop24(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices24(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J24_2{i};
            JB = J24_2{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base2'*base4;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob24{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob24(i) = sqrt(det(relative_jacob24{i}*relative_jacob24{i}'));
        else
            MM_relative_jacob24(i)=0;
        end
    end
    MM_relative_jacob(5)=max(MM_relative_jacob24);
else
    MM_relative_jacob(5)=0;
end
% 臂3与臂4
relative_jacob34=cell(1,myLength(coop34));
MM_relative_jacob34 = zeros(1,myLength(coop34));
if ~isempty(coop34)
    parfor i=1:myLength(coop34)
        if length(J34_4{i})==7
            %计算两末端点向量p
            p = coop34recover(i,:)-coop34(i,:);
            matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
            %计算基座臂末端到基座臂基座的旋转矩阵R1
            r_BA = R_BA1{matching_row_indices34(i)};
            %计算两条臂当前关节角下的雅各比矩阵
            JA = J34_3{i};
            JB = J34_4{i};    
            matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
            %计算基座臂末端到操作臂基座的旋转矩阵
            R_AB = base3'*base4;
            R2 = r_BA\R_AB;
            matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
            relative_jacob34{i} = [ matix1*matix2*JA , matix3*JB];
            MM_relative_jacob34(i) = sqrt(det(relative_jacob34{i}*relative_jacob34{i}'));
        else
            MM_relative_jacob34(i)=0;
        end
    end
    MM_relative_jacob(6)=max(MM_relative_jacob34);
else
    MM_relative_jacob(6)=0;
end

%% 
%% 动力学灵巧度计算
MB = diag([0.026 0.026 0.026 0.987e-6,0.987e-6,0.987e-6]);
W_tao=diag(2.*[10 10 10 10 10 10 10 10 10 10 10 10 10 10]);
wG12=zeros(1,myLength(coop12));
wG13=zeros(1,myLength(coop13));
wG14=zeros(1,myLength(coop14));
wG23=zeros(1,myLength(coop23));
wG24=zeros(1,myLength(coop24));
wG34=zeros(1,myLength(coop34));
%臂1&2
parfor i=1:myLength(coop12)
    J_f1=J12_1{i};
    J_f2=J12_2{i};
    
    %定义惯性矩阵M
    MF1=MF12_1{i};
    MF2=MF12_2{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_A + vertex_B)/2;
    D_b1 = [eye(3) -skew(vertex_A - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_B - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG12{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS12(i)=max(diag(S));
    wG12(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end

%臂1&3
parfor i=1:myLength(coop13)
    J_f1=J13_1{i};
    J_f2=J13_3{i};
    
    %定义惯性矩阵M
    MF1=MF13_1{i};
    MF2=MF13_3{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_A + vertex_C)/2;
    D_b1 = [eye(3) -skew(vertex_A - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_C - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG13{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS13(i)=max(diag(S));
    wG13(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end

%臂1&4
parfor i=1:myLength(coop14)
    J_f1=J14_1{i};
    J_f2=J14_4{i};
    
    %定义惯性矩阵M
    MF1=MF14_1{i};
    MF2=MF14_4{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_A + vertex_D)/2;
    D_b1 = [eye(3) -skew(vertex_A - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_D - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG14{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS14(i)=max(diag(S));
    wG14(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end


%臂2&3
parfor i=1:myLength(coop23)
    J_f1=J23_2{i};
    J_f2=J23_3{i};
    
    %定义惯性矩阵M
    MF1=MF23_2{i};
    MF2=MF23_3{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_B + vertex_C)/2;
    D_b1 = [eye(3) -skew(vertex_B - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_C - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG23{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS23(i)=max(diag(S));
    wG23(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end

%臂2&4
parfor i=1:myLength(coop24)
    J_f1=J24_2{i};
    J_f2=J24_4{i};
    
    %定义惯性矩阵M
    MF1=MF24_2{i};
    MF2=MF24_4{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_B + vertex_D)/2;
    D_b1 = [eye(3) -skew(vertex_B - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_D - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG24{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS24(i)=max(diag(S));
    wG24(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end

%臂3&4
parfor i=1:myLength(coop34)
    J_f1=J34_3{i};
    J_f2=J34_4{i};
    
    %定义惯性矩阵M
    MF1=MF34_3{i};
    MF2=MF34_4{i};
    MF=blkdiag(MF1,MF2);
    %定义物体惯性张量
    midpoint_i = (vertex_C + vertex_D)/2;
    D_b1 = [eye(3) -skew(vertex_C - midpoint_i)];
    D_b2 = [eye(3) -skew(vertex_D - midpoint_i)];
    D_f1 = [eye(3) -skew([-0.15 0 0])];
    D_f2 = [eye(3) -skew([ 0.15 0 0])];
    J_cf = blkdiag(D_f1*J_f1,D_f2*J_f2);
    
    D_b=[D_b1;D_b2];
%     Fai = [D_b -J_cf];
    M_ba = inv(D_b*inv(MB)*D_b'+J_cf*inv(MF)*J_cf');
    HF=(-M_ba)*J_cf*inv(MF);
    G_BF=(D_b')*M_ba*J_cf*inv(MF);
    %计算权重矩阵w_tao w_v
    w_v=sqrt(MB);
    
    
    E=eye(6);
    gamma_B=round(w_v*inv(MB)*G_BF*W_tao,4);
    gamma_F=E*(eye(6)-pinv(D_b')*D_b')*HF*W_tao;
%     pgf=pinv(gamma_F);
    

    bili=0.7
    gamma_G=sqrt((1- bili)^2)*gamma_B*round((eye(14)-pinv(gamma_F)*gamma_F),4);
%     gG34{i}=gamma_G;
%     [~,S,~]=svd(gamma_G*gamma_G');
%     maxS34(i)=max(diag(S));
    wG34(i)=sqrt(det(gamma_G(1:3,:)*gamma_G(1:3,:)'));
end
MAX12=max(wG12);
MAX13=max(wG13);
MAX14=max(wG14);
MAX23=max(wG23);
MAX24=max(wG24);
MAX34=max(wG34);



%% 

% 返回值re为最大运动学可操作度
[re, index] = max(MM_relative_jacob);
% 返回值f2为最大运动学可操作度双臂对应的协同体积

%dyna为最大动力学指标
dyna=max([MAX12,MAX13,MAX14,MAX23,MAX24,MAX34]);
if isempty(dyna)
    dyna=100;
end

if re==0                   %若四臂均不存在交集
    f(1) = 100;
    f(2) = 100;
    f(5) = 100;              %f（5）为最大可操作度双臂对应的基座精度
else
    f(1) = -re;
    f(2) = -volume(alpha_coop{index});
    f(5) = -min_K(index);
end
f(4)=-dyna;

end
%f=[运动学可操作度  协同体积  刚度 动力学可操作度 基座精度]
F(t,1:5)=f;

end



%% 


function rowCount = myLength(A)
    % myLength 函数返回矩阵 A 的行数

    % 获取矩阵 A 的行数
    rowCount = size(A, 1);
end

function A1 = shaixuan(A,alpha_shape)
        in_alpha_shape = inShape(alpha_shape, A(:,1), A(:,2), A(:,3));
        A1=A(~in_alpha_shape, :);
end

end
