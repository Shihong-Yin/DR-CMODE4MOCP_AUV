classdef DRCMODECR2 < ALGORITHM
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
            while Algorithm.NotTerminated(Population)
                Offspring  = DEgenerator2(Population,Problem); % 生成后代种群，通过差分进化生成后代
                t          = Problem.FE/Problem.N; % 计算当前的函数评估次数与种群大小的比值
                MaxGen     = Problem.maxFE/Problem.N; % 计算最大进化代数与种群大小的比值
                a          = 0.5*(1-cos((1-t/MaxGen)*pi)); % 计算一个动态调整的参数a
                Population = EnvironmentalSelection([Population,Offspring],Problem.N,a,Problem); % 进行环境选择，融合当前种群和后代种群，并进行选择操作
            end
        end
    end
end