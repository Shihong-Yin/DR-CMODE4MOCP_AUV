clear
clc

% 指定文件夹的路径
folders = {'NSGAII', 'CCMO', 'BiCo', 'CMEGL', 'MSCMO', 'MTCMO', 'URCMO', 'CMOEMT', 'APSEA', 'DRLOSEMCMO', 'DRCMODEDTS', 'DRCMODE'};
Algorithm_name = {'NSGA-II', 'CCMO', 'BiCo', 'CMEGL', 'MSCMO', 'MTCMO', 'URCMO', 'CMOEMT', 'APSEA', 'DRLOS-EMCMO', 'DR-CMODE-DTS', 'DR-CMODE'};

Problem_name = {'MOCP_AUV1', 'MOCP_AUV2', 'MOCP_AUV3', 'MOCP_AUV4'};
Problem_name1 = {'MOCP\_AUV1', 'MOCP\_AUV2', 'MOCP\_AUV3', 'MOCP\_AUV4'};

Numbers = 10; % 记录数据点数
Colors = [228, 60, 47;255, 215, 0;170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255;...
    228, 60, 47;255, 215, 0;170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255]./255;
% % 生成 n 种颜色的 HSV 值
% numColors = length(folders) + 1;  % 定义需要的颜色数量
% hsvColors = [linspace(0, 1, numColors)', ones(numColors, 2)]; % 色调均匀分布，饱和度和亮度固定为 1
% % 转换为 RGB 值
% Colors = hsv2rgb(hsvColors);

LineStyle = {'-.','-.','-.','-.','-.','-.','-','-','-','-','-','-'};
Marker = {'^','s','<','d','>','o','v','p','h','+','x','*'};

for num = 1:4 %%% 测试场景
    average_HV = zeros(length(folders), Numbers);

    % 循环遍历每个文件夹
    for i = 1:length(folders)
        folder = folders{i};

        % 获取文件夹中所有以'MOCP_AUV1'开头的.mat文件
        files = dir(fullfile(folder, [folder '_' Problem_name{num} '*.mat']));

        % 初始化一个数组来存储当前文件夹中的所有HV数据
        HV_values = zeros(Numbers, length(files));

        % 循环遍历每个文件
        for j = 1:length(files)
            % 读取.mat文件
            file_path = fullfile(folder, files(j).name);
            loaded_data = load(file_path);

            % 获取metric.HV数据
            HV_values(:, j) = loaded_data.metric.HV(1:end, 1);
        end

        maxFE = loaded_data.result{Numbers, 1};
        x = linspace(0, maxFE, Numbers+1);
        NFEs = x(1, 2:end);

        % 计算HV数据的平均值
        HV_values(isnan(HV_values)) = 0;
        average_HV(i, :) = mean(HV_values, 2);
    end

    figure
    hold on
    for i = 1:length(folders)
        plot(NFEs,average_HV(i, :),'LineStyle',LineStyle{i},'Marker',Marker{i},'Color',Colors(i,:),'Linewidth',1);
    end
    hold off
    xlabel('Number of function evaluations');
    ylabel('HV'); %%%
    title(Problem_name1{num});

    lgd = legend(Algorithm_name,'Location','southeast','Orientation','vertical');
    lgd.BoxFace.ColorType = 'truecoloralpha';
    lgd.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明

    box on
    set(gca,'FontName','Times New Roman','FontSize',15,'Box','on','View',[0 90]);

    xlim([NFEs(1),NFEs(end)])
    set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
    set(gcf,'renderer','painters')
end
