function [Population] = EnvironmentalSelection(Population,N,a,Problem)
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
    
    FrontNo2  = NDSort(Population.objs,0,inf); % 对种群进行第二次非支配排序，不考虑约束
    CrowdDis2 = CrowdingDistance(Population.objs,FrontNo2); % 计算第二次排序的拥挤度

    [~,r2]    = sortrows([FrontNo2',-CrowdDis2']);
    Rp(r2)    = 1 : N1;
    % 根据第二次排序结果计算非支配排序
    
    pro_l = 1-length(find(sum(max(0,Population.cons),2)>0))/N1;% 计算可行性率，pro_l表示满足约束条件的个体的比例
    b     = 1.0/((Problem.FE/Problem.maxFE)^2+1)-0.5; % 计算一个用于调整的参数b
    r     = a*(1-b)+pro_l*b; % 计算排名的融合因子r
    R_sum = (1-r)*Rc+r*Rp; % 融合两个排序的结果
    
    [~,Rank]   = sort(R_sum); % 对融合结果进行排序
    Population = Population(Rank(1:N)); % 选择排序前N个个体作为新的种群
end