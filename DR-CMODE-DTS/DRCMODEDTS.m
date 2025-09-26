classdef DRCMODEDTS < ALGORITHM
% <2025> <multi> <real/integer> <constrained>
% Dual-ranking constrained multi-objective differential evolution

%------------------------------- Copyright --------------------------------
% Copyright (c) 2025 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------
    methods
        function main(Algorithm,Problem)
%             rng(0)
            %% Generate random population
            Population = Problem.Initialization(); % 初始化一个种群
            
            %% Optimization
            C = 100;
            pds = repelem(makedist('Beta'), 4);

            for i = 1:4
                pds(i) = makedist('Beta', 'a', 1, 'b', 1);
            end
            op = zeros(Problem.N, 1);
            countOps = zeros(1, 4);

            pen = 1e3;

            objs = Population.objs;
            cons = Population.cons;
            PF_old = objs + pen * sum(max(cons, 0), 2);

            %% Optimization
            while Algorithm.NotTerminated(Population)
                for i = 1:Problem.N
                    % Bandit-based operator selection
                    op(i) = BetaSample(pds);
                    countOps(op(i)) = countOps(op(i)) + 1;
                end

                Offspring  = DEgenerator2(Population,Problem,op); % 生成后代种群，通过差分进化生成后代
                objs = Population.objs;
                cons = Population.cons;
                PF_new = objs + pen * sum(max(cons, 0), 2);

                % 判定：PF_new <= PF_old（逐目标都不差）且 存在一目标 PF_new < PF_old（严格更好）
                notWorse = all(PF_new <= PF_old, 2);          % 所有目标不更差
                strictBetter = any(PF_new <  PF_old, 2);      % 至少一个目标严格更好
                improved = notWorse & strictBetter;           % 逻辑向量，长度为种群规模
                improve_flag = double(improved);
                
                PF_old = PF_new;   % 用于下一代比较

                for i = 1:Problem.N
                    r = improve_flag(i);
                    a = pds(op(i)).a; b = pds(op(i)).b;
                    if a + b < C
                        a = a + r;
                        b = b + 1 - r;
                    else
                        a = (a + r) * C / (C + 1);
                        b = (b + 1 - r) * C / (C + 1);
                    end
                    pds(op(i)).a = a;
                    pds(op(i)).b = b;
                end

                t          = Problem.FE/Problem.N; % 计算当前的函数评估次数与种群大小的比值
                MaxGen     = Problem.maxFE/Problem.N; % 计算最大进化代数与种群大小的比值
                a          = 0.5*(1-cos((1-t/MaxGen)*pi)); % 计算一个动态调整的参数a
                Population = EnvironmentalSelection([Population,Offspring],Problem.N,a,Problem); % 进行环境选择，融合当前种群和后代种群，并进行选择操作
            end
        end
    end
end
