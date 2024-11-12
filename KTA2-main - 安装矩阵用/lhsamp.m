function S = lhsamp(m, n)
% 生成第一代用于采样的个体的变量
%LHSAMP  Latin hypercube distributed random numbers
%
% Call:    S = lhsamp
%          S = lhsamp(m)
%          S = lhsamp(m, n)
%
% m : number of sample points to generate, if unspecified m = 1
% n : number of dimensions, if unspecified n = m
%
% S : the generated n dimensional m sample points chosen from
%     uniform distributions on m subdivions of the interval (0.0, 1.0)

% hbn@imm.dtu.dk  
% Last update April 12, 2002

if nargin < 1, m = 1; end
if nargin < 2, n = m; end

S = zeros(m,n);
for i = 1 : n
    % 生成m×n维的随机均匀排列的采样点
    % m个随机数组成的向量+0至m-1的随机排列，除以m确保随机数在0-1之间是随机分布的
  S(:, i) = (rand(1, m) + (randperm(m) - 1))' / m;
end