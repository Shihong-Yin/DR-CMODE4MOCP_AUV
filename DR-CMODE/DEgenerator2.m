function [Offspring] = DEgenerator2(Population,Problem)
%Search operator based on differential evolution

%------------------------------- Copyright --------------------------------
% Copyright (c) 2025 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------
    FrontNo = NDSort(Population.objs,Population.cons,1); % 对当前种群进行非支配排序
    [N,D]   = size(Population(1).decs); % 获取种群个体的数量N和每个个体的维度D
    trial   = zeros(1*Problem.N,D); % 初始化一个空矩阵用于存储试验解
    
    for i = 1 : Problem.N
    
        % 随机选择一个来自CDP排序级别1的个体作为最优个体
        index_No1  = find(FrontNo==1);
        r          = floor(rand*length(index_No1))+1;
        best_index = index_No1(r);
    
        % 随机选择三个互不相同的个体
        indexset    = 1 : Problem.N;
        indexset(i) = [];
        r1  = floor(rand*(Problem.N-1))+1;
        xr1 = indexset(r1);
        indexset(r1) = [];
        r2  = floor(rand*(Problem.N-2))+1;
        xr2 = indexset(r2);
        indexset(r2) = [];
        r3  = floor(rand*(Problem.N-3))+1;
        xr3 = indexset(r3);
    
        if rand <= 0.5
            % 如果随机数小于等于0.5，执行Mutation操作1或Mutation操作2
            CR = 0.1;
            if rand < 0.5
                F_pool = [0.6,0.8,1.0];
                F      = F_pool(randi(3));
                v      = Population(xr1).decs+rand*(Population(best_index).decs-Population(xr1).decs)+F*(Population(xr2).decs-Population(xr3).decs);
                % Mutation操作1
            else
                F_pool = [0.1,0.8,1.0];
                F      = F_pool(randi(3));
                v      = Population(xr1).decs+F*(Population(i).decs-Population(xr1).decs)+F*(Population(xr2).decs-Population(xr3).decs);
                % Mutation操作2
            end
    
            % 边界修复操作
            Lower = repmat(Problem.lower,N,1);
            Upper = repmat(Problem.upper,N,1);
            v     = min(max(v,Lower),Upper);
    
            % 交叉操作
            Site   = rand(N,D) < CR;
            j_rand = floor(rand * D) + 1;
            Site(1, j_rand) = 1;
            Site_  = 1-Site;
            trial(i, :) = Site.*v+Site_.*Population(i).decs;
            % 通过交叉和变异操作生成候选解
    
        else
            % 如果随机数大于0.5，执行Mutation操作3或Mutation操作4
            if rand < 0.5
                F_pool = [0.6,0.8,1.0];
                F      = F_pool(randi(3));
                v      = Population(i).decs+rand*(Population(xr1).decs-Population(i).decs)+F*(Population(xr2).decs-Population(xr3).decs);
                % Mutation操作3
            else
                F_pool = [0.1,0.8,1.0];
                F      = F_pool(randi(3));
                v      = Population(i).decs+F*(Population(best_index).decs-Population(i).decs)+F*(Population(xr1).decs-Population(xr2).decs);
                % Mutation操作4
            end
            % 边界修复操作
            Lower = repmat(Problem.lower,N,1);
            Upper = repmat(Problem.upper,N,1);
            trial(i, :) = min(max(v,Lower),Upper);
            % 将生成的解限制在问题的边界内
        end
    end
    % 对生成的后代进行评估
    Offspring = trial;
    Offspring = Problem.Evaluation(Offspring);
end