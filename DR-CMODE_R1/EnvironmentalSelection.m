function [Population] = EnvironmentalSelection(Population,N)
% Environmental selection

%------------------------------- Copyright --------------------------------
% Copyright (c) 2025 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------
    [N1,~]       = size(Population.objs); % 获取种群的目标值数量
    [FrontNo1,~] = NDSort(Population.objs,Population.cons,inf); % 对种群进行非支配排序
    CrowdDis1    = CrowdingDistance(Population.objs,FrontNo1); % 计算种群中每个个体的拥挤度

    [~,r1]       = sortrows([FrontNo1',-CrowdDis1']);
    Rc(r1)       = 1 : N1;
    % 根据排序结果计算CDP排序
    
    R_sum = Rc; % 只用CDP排序
    
    [~,Rank]   = sort(R_sum); % 对融合结果进行排序
    Population = Population(Rank(1:N)); % 选择排序前N个个体作为新的种群
end