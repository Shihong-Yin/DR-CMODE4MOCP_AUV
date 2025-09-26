%% collect_max_objs.m
% 扫描当前目录下的各算法子文件夹，批量读取 *_MOCP_AUV4_*.mat，
% 对每个文件的 result{10,2}.objs 计算“每列最大值”，
% 然后在同一算法的多次运行之间再取“列最大值中的最大值”。
% 结果汇总保存为 CSV，并在命令行打印概览。

clc; clear;

% ======== 可按需修改的参数 ========
rootDir     = pwd;         % 顶层目录：默认就是当前目录
problemName = 'MOCP_AUV4';    % 问题名用于匹配文件
filePattern = ['*_' problemName '_*.mat'];
% ================================

fprintf('根目录：%s\n', rootDir);

% 自动检测算法子目录（只取当前目录下的子文件夹）
d = dir(rootDir);
isAlg = [d.isdir] & ~ismember({d.name},{'.','..'});
algDirs = {d(isAlg).name};
if isempty(algDirs)
    error('未在 %s 下发现任何算法子目录。', rootDir);
end
fprintf('检测到算法文件夹：%s\n', strjoin(algDirs, ', '));

% 汇总容器
allAlgRows   = {};     % 每个算法一行：Algorithm, Obj1_Max, Obj2_Max, ...
allRunRows   = {};     % 每次运行一行：Algorithm, RunFile, Obj1_Max, Obj2_Max, ...
colCountSeen = [];     % 记录各算法目标维数，便于统一列名

for i = 1:numel(algDirs)
    alg = algDirs{i};
    algPath = fullfile(rootDir, alg);
    if ~isfolder(algPath), continue; end

    files = dir(fullfile(algPath, filePattern));
    if isempty(files)
        warning('算法 %s 未找到匹配 %s 的 .mat 文件。', alg, filePattern);
        continue;
    end

    perRunMax = [];            % runs × M
    runNames  = strings(0,1);

    fprintf('\n==== 处理算法：%s ====\n', alg);

    for k = 1:numel(files)
        fpath = fullfile(files(k).folder, files(k).name);
        try
            load(fpath);                % 加载 .mat

            objs = result{10,2}.objs;       % N × M
            if ~ismatrix(objs)
                warning('objs 非二维矩阵：%s（跳过）', fpath);
                continue;
            end
            colMax = nanmax(objs, [], 1);   % 1 × M（忽略 NaN）
            perRunMax = [perRunMax; colMax];
            runNames(end+1,1) = string(files(k).name);

            % 记录逐次运行行
            allRunRows = [allRunRows; [{alg}, {files(k).name}, num2cell(colMax)]];

        catch ME
            warning('读取失败：%s\n原因：%s', fpath, ME.message);
        end
    end

    if isempty(perRunMax)
        warning('算法 %s 没有成功解析的运行结果。', alg);
        continue;
    end

    overallMax = max(perRunMax, [], 1);   % 1 × M
    M = numel(overallMax);
    colCountSeen(end+1) = M;

    fprintf('运行次数：%d，目标维数 M：%d\n', size(perRunMax,1), M);
    fprintf('该算法在多次运行间的列最大值（overall）：\n');
    disp(overallMax);

    % 算法汇总行
    allAlgRows = [allAlgRows; [{alg}, num2cell(overallMax)]];
end

% 若没有任何数据，直接结束
if isempty(allAlgRows)
    error('没有可汇总的数据，请检查目录与文件命名。');
end

% 统一列名（根据出现过的最大维数）
maxM = max(colCountSeen);
algVarNames = [{'Algorithm'}, arrayfun(@(j) sprintf('Obj%d_Max', j), 1:maxM, 'UniformOutput', false)];
runVarNames = [{'Algorithm','RunFile'}, arrayfun(@(j) sprintf('Obj%d_Max', j), 1:maxM, 'UniformOutput', false)];

% 将 cell 填齐（有的算法/运行维数可能更少）
allAlgRows = padRows(allAlgRows, numel(algVarNames));
allRunRows = padRows(allRunRows, numel(runVarNames));

% 生成表并保存
AlgTable = cell2table(allAlgRows, 'VariableNames', algVarNames);
RunTable = cell2table(allRunRows, 'VariableNames', runVarNames);

outCsv1 = fullfile(rootDir, sprintf('MaxByAlgorithm_%s.csv', problemName));
outCsv2 = fullfile(rootDir, sprintf('MaxByRun_%s.csv', problemName));
writetable(AlgTable, outCsv1);
writetable(RunTable, outCsv2);

fprintf('\n已保存算法级汇总：%s\n', outCsv1);
fprintf('已保存逐次运行汇总：%s\n', outCsv2);

% 把结果放到工作区便于查看
assignin('base', 'MaxByAlgorithmTable', AlgTable);
assignin('base', 'MaxByRunTable',       RunTable);

%% ======= 辅助函数：将不等长行补齐为同列数 =======
function C = padRows(C, targetCols)
    for ii = 1:size(C,1)
        if numel(C(ii,:)) < targetCols
            C(ii, end+1:targetCols) = {[]};
        end
    end
end
