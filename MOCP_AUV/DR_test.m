clear
clc

% 设置 P_t 和 P_f 的范围
P_t = linspace(0, 1, 100);  % 迭代进度
P_f = linspace(0, 1, 100);  % 可行解比例

% 计算 gamma
[T, P] = meshgrid(P_t, P_f);
alpha = 0.5 * (1 - cos(pi * (1 - T)));
beta = 1 ./ ( (T).^2 + 1) - 0.5;
gamma = alpha .* (1 - beta) + P .* beta;

% 绘制曲面图
figure('Color','w');
surf(T, P, gamma);

shading interp;  % 平滑曲面颜色

xlabel('Iteration progress (t/T_{max})');
ylabel('Feasible solution proportion (P_f)');
zlabel('Weight (\gamma)');
% title('Influence of P_f and P_t on \gamma');
colorbar;

% 美化图形
colormap jet;  % 使用jet颜色图
axis tight;  % 自动调整坐标轴范围
view(45, 30);  % 设置视角，提升效果
set(gca,'FontName','Times New Roman','FontSize',10);
set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')
