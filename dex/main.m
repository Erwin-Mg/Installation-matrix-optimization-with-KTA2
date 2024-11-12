% Code for calculating the dexterity of three continuum robots combining
% concentric tube mechanisms and cable driven mechanisms.
%
% Please Refer to:
%
% Liao Wu, Ross Crawford, Jonathan Roberts. Dexterity analysis of three 
% 6-DOF continuum robots combining concentric tube mechanisms and cable 
% driven mechanisms. IEEE Robotics and Automation Letters. 2017, 2(2): 
% 514-521.

clear;
clc;
close all;
tic;
%% set up parameters for simulation
N       = 100000;        % number of samples, for quick run
% N       = 100000000;    % number of samples, used in the paper
Ntheta  = 60;           % number of intervals for theta
Nh      = 30;           % number of intervals for h
deltaX  = 10;            % interval of position X
deltaZ  = 10;            % interval of position Z
joint_number = 7;       % 机器人旋转关节数量

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
% robot.teach;
% 在关节空间内随机采样
joint_angel = random ('Uniform',0,1,N,joint_number) * pi;
%% calculate the statistics of the configurations of all the samples
C=zeros(N,5);


% 根据机器人根部位姿乘转换矩阵 R
% robot.base = T;

for i=1:N
    C(i,:)=Config(robot,joint_angel(i,:),Ntheta,Nh,deltaX,deltaZ);
end

[D, ia, ~] = unique(C, 'rows');
% D_indice = [D, ia];                 %保留索引信息
% joint_angel = joint_angel(ia,:);
% D=unique(C,'rows');

% D1 = sortrows(D, [4 5 1 3]);

[E,F]=unique(D(:,1:3),'rows');


[DX,LengthX,longest_x,indice_x] = compute_length(D,1);
[DY,LengthY,longest_y,indice_y] = compute_length(D,2);
[DZ,LengthZ,longest_z,indice_z] = compute_length(D,3);



X_traj = DX(indice_x(1):indice_x(end),:);
Y_traj = DY(indice_y(1):indice_y(end),:);
Z_traj = DZ(indice_z(1):indice_z(end),:);

X_ma_null = zeros(longest_x,1);
for i=1:size(X_traj,1)
    [~,row_indice] = ismember (X_traj(i,:),D,"rows");
    row_indice = ia(row_indice);
    X_ma_null(i) = manu(robot,joint_angel(row_indice,:),0);
end

toc;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G=zeros(size(F,1),1);
for i=1:size(F,1)-1
    G(i)=F(i+1)-F(i);
end
G(size(F,1))=size(D,1)-F(size(F,1))+1;

H=zeros(size(G,1),4);
% 总灵巧度
H(:,1)=G/Ntheta/Nh;
% 计算三个方向灵巧度
for i=1:size(G,1)-1
    H(i,2)=sum(abs(cos(D(F(i):F(i+1)-1,3)*2*pi/Ntheta).*sqrt(1-(D(F(i):F(i+1)-1,4)*2/Nh).^2)))/Ntheta/Nh;
    H(i,3)=sum(abs(sin(D(F(i):F(i+1)-1,3)*2*pi/Ntheta).*sqrt(1-(D(F(i):F(i+1)-1,4)*2/Nh).^2)))/Ntheta/Nh;
    H(i,4)=sum(abs(D(F(i):F(i+1)-1,4)*2/Nh))/Ntheta/Nh;
end
H(size(G,1),2)=sum(abs(cos(D(F(size(G,1)):end,3)*2*pi/Ntheta).*sqrt(1-(D(F(size(G,1)):end,4)*2/Nh).^2)))/Ntheta/Nh;
H(size(G,1),3)=sum(abs(sin(D(F(size(G,1)):end,3)*2*pi/Ntheta).*sqrt(1-(D(F(size(G,1)):end,4)*2/Nh).^2)))/Ntheta/Nh;
H(size(G,1),4)=sum(abs(D(F(size(G,1)):end,4)*2/Nh))/Ntheta/Nh;


%% draw orientation
[a,b]=max(H(:,1));
[ax,~]=max(H(:,2));
[ay,~]=max(H(:,3));
[az,~]=max(H(:,4));
%% 
figure();
hold on;
theta = linspace(0,2*pi,Ntheta+1);
phi = asin(linspace(-1,1,Nh+1));
[theta,phi] = meshgrid(theta,phi);
[xs,ys,zs] = sph2cart(theta,phi,1);
surf(xs,ys,zs);
colormap([1,1,1]);
for j=F(b):F(b+1)-1
    patchx1=cos((D(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)-0.5)*2/Nh)^2);
    patchx2=cos((D(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)-0.5)*2/Nh)^2);
    patchx3=cos((D(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)+0.5)*2/Nh)^2);
    patchx4=cos((D(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)+0.5)*2/Nh)^2);
    patchy1=sin((D(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)-0.5)*2/Nh)^2);
    patchy2=sin((D(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)-0.5)*2/Nh)^2);
    patchy3=sin((D(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)+0.5)*2/Nh)^2);
    patchy4=sin((D(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((D(j,5)+0.5)*2/Nh)^2);
    patchz1=(D(j,5)-0.5)*2/Nh;
    patchz2=(D(j,5)-0.5)*2/Nh;
    patchz3=(D(j,5)+0.5)*2/Nh;
    patchz4=(D(j,5)+0.5)*2/Nh;
    
    color=zeros(2,2,3);
    color(1,1,:)=[0;1;0];
    color(1,2,:)=color(1,1,:);
    color(2,1,:)=color(1,1,:);
    color(2,2,:)=color(1,1,:);
    surf([patchx1,patchx2;patchx3,patchx4;],[patchy1,patchy2;patchy3,patchy4;],[patchz1,patchz2;patchz3,patchz4;],color);
end
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;
%% 


figure();
hold on;
theta = linspace(0,2*pi,Ntheta+1);
phi = asin(linspace(-1,1,Nh+1));
[theta,phi] = meshgrid(theta,phi);
[xs,ys,zs] = sph2cart(theta,phi,1);
surf(xs,ys,zs);
colormap([1,1,1]);
for j=indice_z(1)
    patchx1=cos((DZ(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)-0.5)*2/Nh)^2);
    patchx2=cos((DZ(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)-0.5)*2/Nh)^2);
    patchx3=cos((DZ(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)+0.5)*2/Nh)^2);
    patchx4=cos((DZ(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)+0.5)*2/Nh)^2);
    patchy1=sin((DZ(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)-0.5)*2/Nh)^2);
    patchy2=sin((DZ(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)-0.5)*2/Nh)^2);
    patchy3=sin((DZ(j,4)-0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)+0.5)*2/Nh)^2);
    patchy4=sin((DZ(j,4)+0.5)*2*pi/Ntheta)*sqrt(1-((DZ(j,5)+0.5)*2/Nh)^2);
    patchz1=(DZ(j,5)-0.5)*2/Nh;
    patchz2=(DZ(j,5)-0.5)*2/Nh;
    patchz3=(DZ(j,5)+0.5)*2/Nh;
    patchz4=(DZ(j,5)+0.5)*2/Nh;
    
    color=zeros(2,2,3);
    color(1,1,:)=[0;1;0];
    color(1,2,:)=color(1,1,:);
    color(2,1,:)=color(1,1,:);
    color(2,2,:)=color(1,1,:);
    surf([patchx1,patchx2;patchx3,patchx4;],[patchy1,patchy2;patchy3,patchy4;],[patchz1,patchz2;patchz3,patchz4;],color);
end
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;
%% calculate the dexterity indices



% dexterity
D_t = sum(H(:,1))/size(H,1);
D_r = sum(H(:,2))/size(H,1);
D_c = sum(H(:,3))/size(H,1);
D_a = sum(H(:,4))/size(H,1);

disp ("total dexterity: " + D_t)                % total dexterity
disp ("radial dexterity: " + D_r)               % radial dexterity
disp ("circumferential dexterity: " + D_c)      % circumferential dexterity
disp ("axial dexterity: " + D_a)                % axial dexterity

% workspace
W=zeros(1,floor(a/0.05)+1);

for k=0:floor(a/0.05)
    W(k+1)=size(find(H(:,1)>=k*0.05),1)*deltaX*deltaZ;
end
disp ("workspace: " + W(1))                     % workspace

% dexterous workspace
W_D = round(D_t * W(1), 0);

disp ("dexterous workspace: " + W_D)            % dexterous workspace

% max dexterity
disp ("max total dexterity: " + a)              % max total dexterity
disp ("max radial dexterity: " + ax)            % max radial dexterity
disp ("max circumferential dexterity: " + ay)   % max circumferential dexterity
disp ("max axial dexterity: " + az)             % max axial dexterity
x_mean = zeros(length(LengthX),1);
y_mean = zeros(length(LengthY),1);
z_mean = zeros(length(LengthZ),1);
% 
% for i = 1:length(LengthX)
%     x_mean(i) = LengthX{i}.length;
% end
% x_mean = mean(x_mean);
% 
% for i = 1:length(LengthY)
%     y_mean(i) = LengthY{i}.length;
% end
% y_mean = mean(y_mean);
% 
% for i = 1:length(LengthZ)
%     z_mean(i) = LengthZ{i}.length ;
% end
% z_mean = mean(z_mean);


toc





