function [PopDec,P] =  funfun(L)

global  D lower upper encoding N PopCon cons cons_flag  M

    %% Initialization
    D = 12;      %决策变量个数
    lower    = zeros(1,D)-L/2;    %决策变量下界
    upper    = zeros(1,D)+L/2;     %决策变量上界
    encoding = 'real';  %real-实数编码
    switch encoding
        case 'binary'
            PopDec = randi([0,1],N,D);
        case 'permutation'
            [~,PopDec] = sort(rand(N,D),2);
        otherwise
            PopDec = unifrnd(repmat(lower,N,1),repmat(upper,N,1));
    end
    cons = zeros(size(PopDec,1),1);
    cons_flag = 1;
    PopCon = cons;
    P = UniformPoint(N,M)/2;
%     %% Calculate objective values
%     switch name
%         case 'DTLZ1'
%             g      = 100*(D-M+1+sum((PopDec(:,M:end)-0.5).^2-cos(20.*pi.*(PopDec(:,M:end)-0.5)),2));
%             PopObj = 0.5*repmat(1+g,1,M).*fliplr(cumprod([ones(N,1),PopDec(:,1:M-1)],2)).*[ones(N,1),1-PopDec(:,M-1:-1:1)];
%             P = UniformPoint(N,M)/2;
%         case 'DTLZ2'
%             g      = sum((PopDec(:,M:end)-0.5).^2,2);
%             PopObj = repmat(1+g,1,M).*fliplr(cumprod([ones(size(g,1),1),cos(PopDec(:,1:M-1)*pi/2)],2)).*[ones(size(g,1),1),sin(PopDec(:,M-1:-1:1)*pi/2)];
%             P = UniformPoint(N,M);
%             P = P./repmat(sqrt(sum(P.^2,2)),1,M);
%         case 'DTLZ3'
%             g      = 100*(D-M+1+sum((PopDec(:,M:end)-0.5).^2-cos(20.*pi.*(PopDec(:,M:end)-0.5)),2));
%             PopObj = repmat(1+g,1,M).*fliplr(cumprod([ones(N,1),cos(PopDec(:,1:M-1)*pi/2)],2)).*[ones(N,1),sin(PopDec(:,M-1:-1:1)*pi/2)];
%             P = UniformPoint(N,M);
%             P = P./repmat(sqrt(sum(P.^2,2)),1,M);
%     end

   %% Sample reference points on Pareto front
    %P = UniformPoint(N,obj.Global.M)/2;
end