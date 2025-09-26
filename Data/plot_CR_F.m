% clear
% clc
% 
% % 提取问题名称和数值矩阵
% problems = {'MOCP\_AUV1', 'MOCP\_AUV2', 'MOCP\_AUV3', 'MOCP\_AUV4'};
% values = [0.23543 	0.23431 	0.23523 	0.22673 	0.22520 	0.22397 	0.22057 	0.22325 	0.22104 	0.22295
%     0.22770 	0.23323 	0.22834 	0.22648 	0.22448 	0.22303 	0.22150 	0.21897 	0.21497 	0.21696
%     0.20846 	0.21947 	0.21410 	0.21166 	0.20666 	0.20736 	0.20678 	0.19784 	0.20441 	0.20157
%     0.17506 	0.17281 	0.17299 	0.17396 	0.17244 	0.17184 	0.17118 	0.16728 	0.16857 	0.16475
%     ]; % CR 数据
% 
% x_data = [0.05 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9];
% % 绘制折线图
% figure;
% hold on;
% set(gcf,'Position',[100 100 600 400]); % 设置画布大小
% 
% % 定义颜色和标记
% colors = [170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255]./255;
% markers = {'o','s','^','d','>','<'};
% 
% for i = 1:size(values,1)
%     plot(x_data, values(i,:), ...
%         '-o', ...
%         'Color', colors(i,:), ...
%         'Marker', markers{mod(i-1,length(markers))+1}, ...
%         'LineWidth', 1.2, ...
%         'MarkerSize', 6, ...
%         'DisplayName', problems{i});
% end
% 
% % 设置横轴
% xlabel('CR', 'FontSize', 12);
% 
% % 设置纵轴
% ylabel('HV', 'FontSize', 12);
% 
% % 图例
% lgd = legend(problems,'Location','northeast','Orientation','horizontal');
% lgd.BoxFace.ColorType = 'truecoloralpha';
% lgd.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
% 
% % 网格和美化
% grid on;
% ax = gca;
% ax.FontName = 'Times New Roman';
% ax.FontSize = 12;
% ax.Box = 'on';
% ax.GridLineStyle = '--';   % 将网格改为虚线
% set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
% set(gcf,'renderer','painters')
% 
% hold off;

%%
clear
clc

% 提取问题名称和数值矩阵
problems = {'MOCP\_AUV1', 'MOCP\_AUV2', 'MOCP\_AUV3', 'MOCP\_AUV4'};
values = [0.093894 	0.081294 	0.139780 	0.175040 	0.210510 	0.224310 	0.218430 	0.220320 	0.219590 	0.208360
    0.145130 	0.125850 	0.152190 	0.184940 	0.207620 	0.212720 	0.220770 	0.215680 	0.208050 	0.210730
    NaN	NaN	0.080569 	0.120370 	0.174520 	0.214520 	0.218680 	0.210470 	0.206620 	0.198140
    0.117190 	0.084368 	0.109570 	0.145200 	0.162950 	0.169080 	0.170040 	0.166890 	0.165950 	0.161600
    ]; % F 数据

x_data = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
% 绘制折线图
figure;
hold on;
set(gcf,'Position',[100 100 600 400]); % 设置画布大小

% 定义颜色和标记
colors = [170, 71, 188;76, 176, 80;255, 140, 0;30, 144, 255]./255;
markers = {'o','s','^','d','>','<'};

for i = 1:size(values,1)
    plot(x_data, values(i,:), ...
        '-o', ...
        'Color', colors(i,:), ...
        'Marker', markers{mod(i-1,length(markers))+1}, ...
        'LineWidth', 1.2, ...
        'MarkerSize', 6, ...
        'DisplayName', problems{i});
end

% 设置横轴
xlabel('F', 'FontSize', 12);

% 设置纵轴
ylabel('HV', 'FontSize', 12);

% 图例
lgd = legend(problems,'Location','northeast','Orientation','horizontal');
lgd.BoxFace.ColorType = 'truecoloralpha';
lgd.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明

% 网格和美化
grid on;
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 12;
ax.Box = 'on';
ax.GridLineStyle = '--';   % 将网格改为虚线
set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')

hold off;