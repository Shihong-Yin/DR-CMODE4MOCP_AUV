clear
clc

% 设置文件夹路径
folder_path = 'DRCMODEDTS';

% 获取文件夹下所有的.mat文件
file_list = dir(fullfile(folder_path, '*.mat'));

% 初始化最大HV值和对应的文件索引
max_hv = -inf;
max_hv_index = 0;

Numbers = 10; % 记录数据点数

% 循环遍历.mat文件
for i = 1:numel(file_list)
    file_name = file_list(i).name; % 获取文件名
    
    % 如果文件名包含"MOCP_AUV1"，则导入该文件并读取metric.HV(end,1)
    if contains(file_name, 'MOCP_AUV3')
        % 导入.mat文件
        data = load(fullfile(folder_path, file_name));
        
        % 读取metric.HV(end,1)
        hv_value = data.metric.HV(end, 1);
        
        % 更新最大HV值和对应的文件索引
        if hv_value > max_hv
            max_hv = hv_value;
            max_hv_index = i;
        end
    end
end

% 使用最大HV值对应的文件索引加载对应的.mat文件
if max_hv_index > 0
    max_hv_file_name = file_list(max_hv_index).name;
    max_hv_data = load(fullfile(folder_path, max_hv_file_name));

    all_best_Solution = cell(1,Numbers);
    
    % 循环遍历1到Numbers行
    for j = 1:Numbers
        % 提取第i行，第2列的objs数据并存储到all_best_PF中
        best_PF = max_hv_data.result{j, 2}.objs;
    
        % 计算每列的最小值和最大值
        min_values = min(best_PF);
        max_values = max(best_PF);

        % 按列归一化到[0,1]
        normalized_PF = (best_PF - min_values) ./ (max_values - min_values);
        [~,index] = min(normalized_PF,[],1);
        [value1,index1] = min(sum(normalized_PF,2));

        best_Dec = max_hv_data.result{j, 2}.decs;

        % 保留最好的几个解决方案
        best_Solution(1,:) = best_Dec(index(1),:);
        best_Solution(2,:) = best_Dec(index(2),:);
        best_Solution(3,:) = best_Dec(index(3),:);
        best_Solution(4,:) = best_Dec(index1,:);

        all_best_Solution{j} = best_Solution;
    end

    save DRCMODEDTS_Solution3 all_best_Solution
    disp(value1)
else
    disp('未找到包含MUCP4的.mat文件');
end
