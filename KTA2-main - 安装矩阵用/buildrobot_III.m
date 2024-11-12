function [robotArm] = buildrobot_III(~)
%BUILDROBOT 此处显示有关此函数的摘要
%   此处显示详细说明
%   关节惯性矩阵

%% 实物机械臂运动学、动力学参数
% K=1e-6;
% M1 = K.*[658.533  -0.059     -0.147;
%         -0.059    637.417    -1.79;
%         -0.147    -1.790     551.364];
% 
% M2 = K.*[658.533   -0.059     -0.147;
%          -0.059    637.417    -1.79;
%          -0.147    -1.79      551.364];
% 
% M3 = K.*[49221   0          0;
%          0       48995.7    -692.634;
%          0      -692.634    1750.57];
% 
% M4 = K.*[12380    0.014        -0.460;
%          0.014    12271.826    -611.612;
%          -0.460   -611.612      912.554];
% 
% M5 = K.*[325.089 -0.028  -0.007;
%          -0.028  310.289 -2.343;
%          -0.007  -2.343  271.049];
% 
% M6 = K.*[325.089 -0.028   -0.007;
%         -0.028   310.289  -2.343;
%         -0.007   -2.343   271.049];
% 
% M7 = K.*[2288.453   3.193      -89.080;
%          3.193      2177.133   2.931;
%          -89.080    2.931      595.021];
% 
% %        theta   d/Z      a/X       alpha     offset
% SL1=Link([0      0       0        -pi/2      0    ],'offset',pi/2,'modified');
% SL1.m=0.518;
% SL1.r=[0 0 0];
% SL1.I=M1;
% SL1.Jm=0.0002;
% SL1.G=-62.61;
% 
% SL2=Link([0      0.098        0        pi/2       0    ],'offset',pi/2,'modified');
% SL2.m=0.518;
% SL2.r=[0 0 0.098];
% SL2.I=M2;
% SL2.Jm=0.0002;
% SL2.G=107.8;
% 
% SL3=Link([0      0.098        0        -pi/2      0    ],'modified');
% SL3.m=1.5;
% SL3.r=[0 0 0.202];
% SL3.I=M3;
% SL3.Jm=0.0002;
% SL3.G=-53.71;
% 
% 
% SL4=Link([0      0          0.404     0          0    ],'modified');
% SL4.m=0.832;
% SL4.r=[0 0 0.15];
% SL4.I=M4;
% SL4.Jm=3.3e-05;
% SL4.G=76.04;
% 
% 
% SL5=Link([0      0.194       0.30       0          0    ],'modified');
% SL5.m=0.329;
% SL5.r=[0 0 0];
% SL5.I=M5;
% SL5.Jm=3.3e-05;
% SL5.G=71.92;
% 
% SL6=Link([0      0.086        0        pi/2       0    ],'modified');
% SL6.m=0.329;
% SL6.r=[0 0 0];
% SL6.I=M6;
% SL6.Jm=3.3e-05;
% SL6.G=76.69 ;
% 
% SL7=Link([0      0.216       0        pi/2       0    ],'modified');
% SL7.m=0.506;
% SL7.r=[0 0 0.049];
% SL7.I=M7;
% SL7.Jm=3.3e-05;
% SL7.G=1 ;
% 
% 
% SL2.offset=pi/2;
% SL3.offset=-pi/2;
% SL5.offset=pi;
% SL6.offset=-pi/2;
%% 模块化机械臂运动学、动力学参数

M1 = [0.00173158  0     0;
         0    0.00171494    0;
         0    0     0.00048433];

M2 = [0.050286   0.000168     -0.01942648;
      0.000168	0.0581253	-0.00034533;
      -0.01942648	 -0.00034533	 0.0087718];

M3 = [0.00173158  0     0;
         0    0.00171494    0;
         0    0     0.00048433];

M4 = [0.08766768	-0.00010697	-0.07703165;
      -0.00010697	0.15668948	0.00011963;
      -0.07703165	0.00011963	0.07044346];

M5 = [0.00010578	  0	  0;
      0	6.978e-05	0;
      0	 0	0.00012401];

M6 = [0.00173158  0     0;
      0    0.00171494    0;
      0    0     0.00048433];


M7 = [ 3.844e-05	0	 0;
       0	 4.272e-05	 0;
       0	 0	 3.896e-05];

%        theta   d/Z      a/X       alpha     offset
SL1=Link([0      0.09744   0          0       0],'offset',pi/2,'modified');
SL1.m=1.514;
SL1.r=[0 0 0];
SL1.I=M1;
SL1.Jm=0.0002;
SL1.G=-62.61;

SL2=Link([0      0         0          pi/2   0],'offset',pi/2,'modified');
SL2.m=1.822;
SL2.r=[0 0 0.098];
SL2.I=M2;
SL2.Jm=0.0002;
SL2.G=107.8;

SL3=Link([0      0.39911    0         pi/2      0],'modified');
SL3.m=1.514;
SL3.r=[0 0 0.202];
SL3.I=M3;
SL3.Jm=0.0002;
SL3.G=-53.71;


SL4=Link([0      0          0        pi/2          0],'modified');
SL4.m=3.24;
SL4.r=[0 0 0.15];
SL4.I=M4;
SL4.Jm=3.3e-05;
SL4.G=76.04;


SL5=Link([0      0         0.44284     pi/2       0],'modified');
SL5.m=0.171;
SL5.r=[0 0 0];
SL5.I=M5;
SL5.Jm=3.3e-05;
SL5.G=71.92;

SL6=Link([0      0.1344    0          pi/2        0  ],'modified');
SL6.m=1.514; 
SL6.r=[0 0 0];
SL6.I=M6;
SL6.Jm=3.3e-05;
SL6.G=76.69 ;

SL7=Link([0      0         0           pi/2       0],'modified');
SL7.m=0.13;
SL7.r=[0 0 0.049];
SL7.I=M7;
SL7.Jm=3.3e-05;
SL7.G=1 ;

SL2.offset=pi;
SL4.offset=pi/2;
SL5.offset=pi/2;
SL6.offset=pi/2;

q(1).qlim=[-180,180]/180*pi;
q(2).qlim=[-180,180]/180*pi;
q(3).qlim=[-180,180]/180*pi;
q(4).qlim=[-180,180]/180*pi;
q(5).qlim=[-180,180]/180*pi;
q(6).qlim=[-180,180]/180*pi;
q(7).qlim=[-180,180]/180*pi;


% 创建单臂机器人
robotArm1 = SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7], 'name', 'Arm1');
robotArm2 = SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7], 'name', 'Arm1');
robotArm3 = SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7], 'name', 'Arm1');
robotArm4 = SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7], 'name', 'Arm1');
robotArm1.gravity=[0 0 0];
robotArm2.gravity=[0 0 0];
robotArm3.gravity=[0 0 0];
robotArm4.gravity=[0 0 0];
robotArm1.tool.t=[0.05 0 0]';
robotArm2.tool.t=[0.05 0 0]';
robotArm3.tool.t=[0.05 0 0]';
robotArm4.tool.t=[0.05 0 0]';
robotArm  = {robotArm1,robotArm2,robotArm3,robotArm4}; 

end

