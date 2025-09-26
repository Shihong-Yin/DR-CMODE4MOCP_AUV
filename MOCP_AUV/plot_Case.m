clear
% clc
close all

Numbers = 10; % 记录数据点数

for problemNum = 3 %%% 测试问题
    algorithmName = 'DRCMODEDTS';
    problemName = ['MOCP\_AUV',num2str(problemNum)];
    solutionName = ['DRCMODEDTS_Solution',num2str(problemNum)];
    load(solutionName)

    best_decs = all_best_Solution{Numbers};

    for i = 4
        best_dec = best_decs(i, :);
        [Fitness, Constraint] = visualize_Path(problemNum, best_dec, algorithmName, problemName);

        Constraint(Constraint < 0) = 0;
        CV = sum(Constraint, 2);
        if CV == 0
            fprintf('时间: %.4f ', Fitness(1));
            fprintf('威胁: %.4f ', Fitness(2));
            fprintf('能耗: %.4f\n', Fitness(3));
        else
            fprintf('不可行解!\n');
        end
    end
end
