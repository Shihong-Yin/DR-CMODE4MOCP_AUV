clear
clc

% 定义文件夹列表
folders = {'NSGAII', 'CCMO', 'BiCo', 'CMEGL', 'MSCMO', 'MTCMO', 'URCMO', 'CMOEMT', 'APSEA', 'DRLOSEMCMO', 'DRCMODEDTS', 'DRCMODE'};
Algorithm_name = {'NSGA-II', 'CCMO', 'BiCo', 'CMEGL', 'MSCMO', 'MTCMO', 'URCMO', 'CMOEMT', 'APSEA', 'DRLOS-EMCMO', 'DR-CMODE-DTS', 'DR-CMODE'};

Problem_name = {'MOCP_AUV1', 'MOCP_AUV2', 'MOCP_AUV3', 'MOCP_AUV4'};
Problem_name1 = {'MOCP\_AUV1', 'MOCP\_AUV2', 'MOCP\_AUV3', 'MOCP\_AUV4'};

Colors = [228, 60, 47;255, 215, 0;170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255;...
    228, 60, 47;255, 215, 0;170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255]./255;
% % 生成 n 种颜色的 HSV 值
% numColors = length(folders) + 1;  % 定义需要的颜色数量
% hsvColors = [linspace(0, 1, numColors)', ones(numColors, 2)]; % 色调均匀分布，饱和度和亮度固定为 1
% % 转换为 RGB 值
% Colors = hsv2rgb(hsvColors);

Marker = {'^','s','<','d','>','o','v','p','h','+','x','*'};

figure
hold on
for num = 1 %%% 测试场景

    Objs = cell(1,length(folders)); % PF目标值
    Cons = cell(1,length(folders)); % PF约束值
    % 循环遍历每个文件夹
    for i = 1:length(folders)
        folder = folders{i};

        % 获取文件夹中所有以'MOCP_AUV1'开头的.mat文件
        files = dir(fullfile(folder, [folder '_' Problem_name{num} '*.mat']));

        % 初始化一个数组来存储当前文件夹中的所有HV数据
        HV_values = zeros(1, length(files));

        % 循环遍历每个文件
        for j = 1:length(files)
            % 读取.mat文件
            file_path = fullfile(folder, files(j).name);
            loaded_data = load(file_path);

            % 获取metric.HV数据
            HV_values(1, j) = loaded_data.metric.HV(end, 1);
        end

        % 找出HV最大的文件索引
        HV_values(isnan(HV_values)) = 0;
        [~, index] = max(HV_values);

        % 读取.mat文件
        file_path = fullfile(folder, files(index).name);
        loaded_data = load(file_path);

        % 获取PF目标值和约束值
        Objs{1,i} = loaded_data.result{end, 2}.objs;
        Cons{1,i} = loaded_data.result{end, 2}.cons;
        PF_mask = find_non_dominated(Objs{1,i}); % 找非支配解索引

        F = Objs{1,i}(PF_mask,:);
        C = Cons{1,i}(PF_mask,:);

        % 可行解判断：所有约束 <= 0
        feasible_idx   = all(C <= 1e-12,2);
        infeasible_idx = ~feasible_idx;

        disp(size(F,1))
        disp(sum(feasible_idx,"all"))

        % 绘制可行解（彩色）
        if any(feasible_idx)
            if i <= length(folders)/2
                scatter3(F(feasible_idx,1), F(feasible_idx,2), F(feasible_idx,3), ...
                    20, 'Marker', Marker{i}, 'MarkerFaceColor', Colors(i,:), ...
                    'MarkerEdgeColor','none','MarkerFaceAlpha',0.5);
            else
                scatter3(F(feasible_idx,1), F(feasible_idx,2), F(feasible_idx,3), ...
                    20, 'Marker', Marker{i}, 'MarkerFaceColor', 'none', ...
                    'MarkerEdgeColor', Colors(i,:));
            end
        end

        % 绘制不可行解（黑色）
        if any(infeasible_idx)
            if i <= length(folders)/2
                scatter3(F(infeasible_idx,1), F(infeasible_idx,2), F(infeasible_idx,3), ...
                    20, 'Marker', Marker{i}, 'MarkerFaceColor','k', ...
                    'MarkerEdgeColor','none','MarkerFaceAlpha',0.5);
            else
                scatter3(F(feasible_idx,1), F(feasible_idx,2), F(feasible_idx,3), ...
                    20, 'Marker', Marker{i}, 'MarkerFaceColor', 'none', ...
                    'MarkerEdgeColor', 'k');
            end
        end
    end

end

view([-90 -90 90])
% grid off
% view([0 0])
% title('Front view');
% view([0 90])
% title('Side view');
% view([90 0])
% title('Top view');

% 添加标签和标题
xlabel('Time');
ylabel('Threat');
zlabel('Energy');
title(Problem_name1{num});
lgd = legend(Algorithm_name,'Location','northwest','Orientation','vertical');
lgd.BoxFace.ColorType = 'truecoloralpha';
lgd.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
grid on
set(gca,'FontName','Times New Roman','FontSize',15,'Box','on');
set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')
