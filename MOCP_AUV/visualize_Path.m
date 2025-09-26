function [Fitness, Constraint] = visualize_Path(problemNum, solution, algorithm, problem)
% 根据传入问题编号和解决方案编码绘制路径规划图
% 根据权重计算协同路径的评价指标

[AUV, ENV] = Environment(problemNum);

AUV_num = AUV.num; % AUV的数量
cruise = AUV.cruise;
initialBattery = AUV.initialBattery;
P_cruise = AUV.P_cruise;
P_turn = AUV.P_turn;
P_climb = AUV.P_climb;
Density = AUV.density; % 插值密度
safeDist = AUV.safeDist;
range = AUV.searchRange;

% 获取环境参数
dim = ENV.dim;
vortices = ENV.vortices;
ThreatArea = ENV.threats;
% 定义影响因子，需根据实际情况调参
k_w = 0.3; % 水流影响因子

Fitness = zeros(size(solution, 1), 3); % 3个目标值
Constraint = zeros(size(solution, 1), 5*AUV_num + 1); %%%%%%%%%%%%%%%%%%%%%

for m = 1:size(solution, 1)
    Agent = solution(m,:);
    multi_Path = decodePath(AUV, Agent); % 将搜索代理解码为航行路径
    segment_Distance = cell(AUV_num, 1); % 存储每个 AUV 路径段的距离
    time_Intervals = cell(AUV_num, 1); % 存储每个 AUV 路径段的时间步长
    path_Unif = zeros(Density, dim, AUV_num); % 存储每个 AUV 的等时间间隔路径点

    %%% 单个AUV的路径约束
    singleFitness = zeros(AUV_num, 2); %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mobilityCon = zeros(1,AUV_num);
    boundsCon = zeros(1,AUV_num);
    threatCon = zeros(1,AUV_num);
    smoothCon = zeros(1,AUV_num);
    energyCon = zeros(1,AUV_num);

    % 1. 任务协同时间
    cruiseSpeed = zeros(AUV_num, Density-1); % 存储巡航速度
    colSpeed = zeros(AUV_num, Density-1);    % 存储协同速度
    cruiseTime = zeros(AUV_num, Density-1);  % 存储巡航时间

    for k = 1:AUV_num
        Path_AUV_k = multi_Path{k}; % 取出第k个AUV的路径
        x1 = Path_AUV_k(1,:);  y1 = Path_AUV_k(2,:);  z1 = Path_AUV_k(3,:); % 全局坐标系中的插值节点

        segment_Distance{k} = sqrt(diff(x1).^2 + diff(y1).^2 + diff(z1).^2); % 计算两点之间的距离

        % 提取水流速度
        waterFlow_x = zeros(size(x1));
        waterFlow_y = zeros(size(y1));
        waterFlow_z = zeros(size(z1));
        % 计算涡流模型的水流速度
        for j = 1:size(vortices,1)
            % 计算每个涡流源的位置对AUV的影响
            x0     = vortices(j,1);
            y0     = vortices(j,2);
            z0     = vortices(j,3);
            lambda = vortices(j,4);
            zeta   = vortices(j,5);

            r2 = (x1 - x0).^2 + (y1 - y0).^2 + (z1 - z0).^2 + eps;  % 避免除零
            factor = 1 - exp(-r2 ./ (zeta^2));
            u = -lambda * (y1 - y0) ./ (2*pi*r2) .* factor;  % x方向速度分量
            v =  lambda * (x1 - x0) ./ (2*pi*r2) .* factor;  % y方向速度分量
            w =  lambda / (pi * zeta^2) * exp(-r2 ./ (zeta^2));  % z方向速度分量

            % 累加多个涡流的影响
            waterFlow_x = waterFlow_x + u;
            waterFlow_y = waterFlow_y + v;
            waterFlow_z = waterFlow_z + w;
        end
        % 将计算得到的水流速度赋值给环境对象
        waterSpeed = [waterFlow_x; waterFlow_y; waterFlow_z];

        for i = 1:Density-1
            segment_vector = Path_AUV_k(:, i+1) - Path_AUV_k(:, i);
            segment_length = norm(segment_vector);
            v_ac_unit = segment_vector / (segment_length + eps);

            env_speed = k_w * waterSpeed(:, i);
            speed_norm_sq = norm(env_speed)^2;
            vw_projection = dot(env_speed, v_ac_unit); % 正负可以判断洋流是推力还是阻力
            disc = vw_projection ^ 2 - (speed_norm_sq - cruise(k) ^ 2);
            if vw_projection >= 0 % 洋流推力
                if disc >= 0
                    vac_mag = vw_projection + sqrt(disc); % 路径段的实际速度大小，受洋流影响
                else
                    mobilityCon(1, k) = max(0, -disc); % 洋流过大，无法达到预期路径
                    vac_mag = vw_projection; % 实际达不到这个速度，只是确保算法不报错
                end
            else % 洋流阻力
                tempFlag = cruise(k) - norm(env_speed);
                if tempFlag >= 0
                    vac_mag = vw_projection + sqrt(disc); % 路径段的实际速度大小，受洋流影响
                else
                    mobilityCon(1, k) = max(0, -tempFlag); % 洋流过大，无法达到预期路径
                    vac_mag = 0.1 * cruise(k); % 实际达不到这个速度，只是确保算法不报错
                end
            end

            cruiseSpeed(k, i) = vac_mag; % 路径段的实际巡航速度
            cruiseTime(k, i) = segment_length / max(vac_mag, eps); % 路径段的实际巡航时间
        end
    end

    minTime = sum(cruiseTime, 2); % 存储所有AUV的最短航行时间
    colTime = max(minTime); % 计算协同时间（即所有AUV中最慢的航行时间）

    % 2. 综合风险代价
    for k = 1:AUV_num
        Path_AUV_k = multi_Path{k};
        % 计算出第 k 个AUV的协同速度，对水中/空中的速度进行比例缩放
        colSpeed(k, :) = cruiseSpeed(k, :) .* minTime(k) ./ colTime;

        % 计算每个位置点的实际时间
        time_Intervals{k} = segment_Distance{k} ./ colSpeed(k, :); % 路径段的巡航时间

        cumsumTime = [0, cumsum(time_Intervals{k})]; % 求累加和
        % 用线性插值去近似第k架AUV的路径
        uniform_time_interval = linspace(0, cumsumTime(end), Density); % 近似线性插值节点
        path_U = interp1(cumsumTime', Path_AUV_k', uniform_time_interval', 'linear'); % 使用interp1函数进行插值
        path_Unif(:,:,k) = path_U;

        x2 = path_U(:, 1)';  y2 = path_U(:, 2)';  z2 = path_U(:, 3)'; % 协同路径的插值节点

        % 边界约束
        a1 = x2(2:end-1) < range(1,1) | x2(2:end-1) > range(1,2);
        a2 = y2(2:end-1) < range(2,1) | y2(2:end-1) > range(2,2);
        a3 = z2(2:end-1) < range(3,1) | z2(2:end-1) > range(3,2);
        boundsCon(1,k) = sum(a1 | a2 | a3); % 约束条件

        % 计算威胁代价
        % 思路：
        % 对路径上的每个点，计算其对于所有威胁区域的D值，取最小D值记为D_min。
        % 若D_min < 1，则该点在某威胁区域内部，根据1-D_min计算威胁约束；
        % 若D_min >= 1，则该点不处于任何威胁区域内部，该点威胁代价为1./(1+(D_min-1).^2)。
        %
        % 最终结果为路径上所有点威胁代价的总和。
        D_min_arr = Inf(1, Density); % 初始化D_min数组，选择最"危险"的威胁区域来定义该点的威胁程度
        for j = 1:size(ThreatArea, 1)
            threat = ThreatArea(j, :); % 取出第 j 个威胁区域
            % 坐标相对偏移
            dx = (x2 - threat(1))/threat(4);
            dy = (y2 - threat(2))/threat(5);
            dz = (z2 - threat(3))/threat(6);
            % 计算D值
            D_vals = (abs(dx).^(2*threat(7))) + (abs(dy).^(2*threat(8))) + (abs(dz).^(2*threat(9)));
            D_min_arr = min(D_min_arr, D_vals); % 更新D_min_arr，存储每个路径点的最小D值
        end
        threatCon(1,k) = sum(1 - D_min_arr(D_min_arr < 1), 2); % 约束条件，路径点在威胁区域内部，位置越里面值越大，数量越多值越大
        threatCost = sum(1./(1 + (D_min_arr(D_min_arr >= 1) - 1).^2), 2); % 路径点在威胁区域外部，位置越远值越小

        singleFitness(k,1) = threatCost; % 安全性

        % 路径不平滑度
        % 思路：
        % 1. 计算相邻点的向量差分，得到速度方向近似值V_i = pathPoints(:,i+1)-pathPoints(:,i)。
        % 2. 再对V_i求差分，以衡量方向和速率的变化率A_i = V_(i+1)-V_i。
        % 3. 将A_i的范数平方求和作为不平滑度度量。
        % 除了计算路径的不平滑度之外，还增加了对每个二阶差分的范数进行检查。
        % 如果某些段的二阶差分范数超过给定的最大允许值 maxNorm，则说明该路径在该段处需要的机动能力超过载具实际可实现的能力，可能无法进行该高难度机动。
        Acc = diff(path_U, 2, 1); % (N-2) x 3 二阶差分（近似加速度向量）
        % 计算每个A_i的范数
        A_norm = vecnorm(Acc, 2, 2); % (N-2) x 1

        maxNorm = pi/2; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 这个参数需要调试
        smoothCon(1,k) = sum(max(A_norm - maxNorm, 0)); % 约束条件，可能不需要
    end

    % 3. 系统总能耗
    for k = 1:AUV_num
        energyCost = 0; % 初始化第 k 个AUV的能耗
        path_U = path_Unif(:,:,k); % 提取协同路径的插值节点
        % 计算每段的方向向量
        segmentPos = diff(path_U, 1, 1); % (N-1) x 3 矩阵，每列一个段向量
        dz = max(segmentPos(:,3), 0); % 爬升高度变化

        for i = 1:Density-1
            dt = time_Intervals{k}(i); % 每段路径的时间步长
            climbRate = dz(i) / dt;    % 爬升速率 m/s

            % 转弯角度计算：需要两段才能计算角度。i>1时可计算转角
            turnAngle = 0;
            if i > 1
                prevPos = segmentPos(i-1, :);
                currentPos = segmentPos(i, :);
                % 计算前后两段的夹角
                cosTheta = (prevPos*currentPos') / (norm(prevPos)*norm(currentPos) + eps);
                cosTheta = max(min(cosTheta,1),-1); % 避免数值误差
                turnAngle = acos(cosTheta);
            end
            % 转弯比例：turnAngle在[0,pi]之间，turnRatio = turnAngle / pi
            turnRatio = turnAngle / pi;

            % 路径段的功率计算
            segmentPower = P_cruise + turnRatio * P_turn + climbRate * P_climb; % 基础巡航功率 + 转向消耗 + 爬升消耗
            segmentEnergy = segmentPower * dt; % 路径段的能量消耗 = segmentPower * dt
            energyCost = energyCost + segmentEnergy;

        end
        % 计算能量裕量
        energyCon(1,k) = max(energyCost - initialBattery(k), 0);
        singleFitness(k, 2) = energyCost; % 能耗
    end

    % 碰撞约束
    distances = zeros(Density, AUV_num*(AUV_num-1)/2); % 多个路径点之间的欧式距离
    for i = 1:Density
        vectors = reshape(path_Unif(i,:,:), dim, AUV_num)';
        distances(i,:) = pdist(vectors); % 计算每对向量之间的欧式距离
    end
    collisionCon = sum(max(safeDist - min(distances,[],2), 0)); % 约束条件

    %%% 计算适应度值
    Fitness(m, :) = [colTime, sum(singleFitness, 1)];
    Constraint(m, :) = [mobilityCon, boundsCon, threatCon, smoothCon, energyCon, collisionCon];
end


%% 绘图
figure;
% figure('Position', [100, 150, 600, 400]);
% subplot(2,3,problemNum)

hold on;
grid on;
axis equal; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title(problem)
view(3);
% camlight;
% lighting gouraud;

% 在图中画出洋流方向
% 创建覆盖整个绘图区域的网格（例如0到100）
x_min = 0; x_max = 100;
y_min = 0; y_max = 100;
z_min = -100; z_max = 0;
resolution = 50; % 网格分辨率

[Xg, Yg, Zg] = meshgrid( ...
    linspace(x_min, x_max, resolution), ...
    linspace(y_min, y_max, resolution), ...
    linspace(z_min, z_max, resolution));
% [xGrid, yGrid, zGrid] = meshgrid(0:5:100, 0:5:100);

% 初始化速度场
U_total = zeros(size(Xg));
V_total = zeros(size(Yg));
W_total = zeros(size(Zg));
% 叠加各涡流源的速度场
for j = 1:size(vortices,1)
    x0     = vortices(j,1);
    y0     = vortices(j,2);
    z0     = vortices(j,3);
    lambda = vortices(j,4);
    zeta   = vortices(j,5);
    % 计算每个网格点到当前涡流源的距离
    r2 = (Xg - x0).^2 + (Yg - y0).^2 + (Zg - z0).^2 + eps;  % 避免除零
    factor = 1 - exp(-r2 ./ (zeta^2));
    % 计算当前涡流源的速度分量
    u = -lambda * (Yg - y0) ./ (2*pi*r2) .* factor;  % x方向速度分量
    v =  lambda * (Xg - x0) ./ (2*pi*r2) .* factor;  % y方向速度分量
    w =  lambda / (pi * zeta^2) * exp(-r2 ./ (zeta^2));  % z方向速度分量
    % 累加到总速度场
    U_total = U_total + u;
    V_total = V_total + v;
    W_total = W_total + w;
end
% 下采样（控制箭头密度）
skip = 4;
Xi = Xg(1:skip:end, 1:skip:end, 1:skip:end);
Yi = Yg(1:skip:end, 1:skip:end, 1:skip:end);
Zi = Zg(1:skip:end, 1:skip:end, 1:skip:end);
Ui = U_total(1:skip:end, 1:skip:end, 1:skip:end);
Vi = V_total(1:skip:end, 1:skip:end, 1:skip:end);
Wi = W_total(1:skip:end, 1:skip:end, 1:skip:end);

% % 速度大小等值面
% spd = sqrt(U_total.^2 + V_total.^2 + W_total.^2);
% isoLevel = max(spd(:)) * 0.3;  % 选择等值面阈值，可调
% p = patch(isosurface(Xg, Yg, Zg, spd, isoLevel));
% isonormals(Xg, Yg, Zg, spd, p);
% p.FaceColor = [0.8 0.8 1];
% p.EdgeColor = 'none';
% alpha(p, 0.3);

% 三维箭头场
quiver3(Xi, Yi, Zi, Ui, Vi, Wi, 1.2, 'Color', [0, 176, 240]./255, 'LineWidth', 0.8, 'MaxHeadSize', 0.5);

% 标出各涡心位置
for k = 1:size(vortices,1)
    plot3(vortices(k,1), vortices(k,2), vortices(k,3), 'ro', 'MarkerSize',8, 'LineWidth',1.5);
end

% 绘制环境威胁
resolution = 30; % 网格分辨率

[Xg, Yg, Zg] = meshgrid( ...
    linspace(x_min, x_max, resolution), ...
    linspace(y_min, y_max, resolution), ...
    linspace(z_min, z_max, resolution));
numThreats = size(ThreatArea, 1);
% 使用isosurface绘制威胁区域表面
for i = 1:numThreats
    threat = ThreatArea(i, :); % 取出第 i 个威胁区域
    % 计算 F 值
    u = (Xg - threat(1))/threat(4);
    v = (Yg - threat(2))/threat(5);
    w = (Zg - threat(3))/threat(6);

    Fgrid = (abs(u).^(2*threat(7))) + (abs(v).^(2*threat(8))) + (abs(w).^(2*threat(9)));

    % 提取F=1的等值面
    p = patch(isosurface(Xg, Yg, Zg, Fgrid, 1));
    set(p, 'FaceColor', 'w', 'EdgeColor', [0.8 0.8 1], 'FaceAlpha', 0.1, 'EdgeAlpha', 0.5);
%     set(p, 'FaceColor', 'w', 'EdgeColor', [178, 178, 255]./255, 'FaceAlpha', 0.3);
end

Colors = hsv(AUV.num); % 使用HSV色图生成10种颜色
makerSize = 10;
for i = 1:AUV.num
    points = path_Unif(:,:,i); % 取出第k个AUV的路径

    % 判断哪些点在障碍物内部
    insideMask = pointsInsideThreats(points, ThreatArea);

    % 将点分成内部与外部两组
    insidePoints = points(insideMask,:);
    outsidePoints = points(~insideMask,:);

    % 绘制内部点：Z>0 空心红圈，Z<=0 实心红圈
%     scatter3(insidePoints(:,1), insidePoints(:,2), insidePoints(:,3), ...
%         makerSize, 'Marker', 'p', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', Colors(i,:), ...
%         'MarkerFaceAlpha', 0.1, 'MarkerEdgeAlpha', 0.3); % 空心半透明标记
    scatter3(insidePoints(:,1), insidePoints(:,2), insidePoints(:,3), ...
        makerSize, 'Marker', 'p', 'MarkerFaceColor',Colors(i,:), 'MarkerEdgeColor', 'none', ...
        'MarkerFaceAlpha', 0.1, 'MarkerEdgeAlpha', 0.3); % 实心半透明标记

    % 绘制外部点：Z>0 空心蓝圈，Z<=0 实心蓝圈
%     scatter3(outsidePoints(:,1), outsidePoints(:,2), outsidePoints(:,3), ...
%         makerSize, 'Marker', 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', Colors(i,:)); % 空心不透明标记
    scatter3(outsidePoints(:,1), outsidePoints(:,2), outsidePoints(:,3), ...
        makerSize, 'Marker', 'o', 'MarkerFaceColor',Colors(i,:), 'MarkerEdgeColor', 'none'); % 实心不透明标记

    % === 安全缓冲区：在路径上按步长下采样绘制半径为 safeDist 的球体 ===
    bufferSkip = max(1, round(size(points,1) / 15));   % 下采样步长：每条轨迹大约画 ~15 个球
    plotSafetyBuffer(points, safeDist, Colors(i,:), bufferSkip);

end

% 创建图例
makerSize = 3;
switch problemNum
    case 1
        h1 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(1,:), 'MarkerEdgeColor', 'none');
        h2 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(2,:), 'MarkerEdgeColor', 'none');
        h3 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(3,:), 'MarkerEdgeColor', 'none');
        h4 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(4,:), 'MarkerEdgeColor', 'none');
        h5 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(5,:), 'MarkerEdgeColor', 'none');
        h6 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(6,:), 'MarkerEdgeColor', 'none');
        hb = scatter3(NaN, NaN, NaN, 80, [0 0 0], 'filled', 'MarkerFaceAlpha', 0.2, 'MarkerEdgeAlpha', 0.2);
        lgd1 = legend([h1, h2, h3, h4, h5, h6, hb], {'AUV 1', 'AUV 2', 'AUV 3', 'AUV 4', 'AUV 5', 'AUV 6', 'Safety buffer'}, 'Location', 'northeast');
        lgd1.BoxFace.ColorType = 'truecoloralpha';
        lgd1.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
    case 2
        h1 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(1,:), 'MarkerEdgeColor', 'none');
        h2 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(2,:), 'MarkerEdgeColor', 'none');
        h3 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(3,:), 'MarkerEdgeColor', 'none');
        h4 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(4,:), 'MarkerEdgeColor', 'none');
        h5 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(5,:), 'MarkerEdgeColor', 'none');
        h6 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(6,:), 'MarkerEdgeColor', 'none');
        h7 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(7,:), 'MarkerEdgeColor', 'none');
        h8 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(8,:), 'MarkerEdgeColor', 'none');
        hb = scatter3(NaN, NaN, NaN, 80, [0 0 0], 'filled', 'MarkerFaceAlpha', 0.2, 'MarkerEdgeAlpha', 0.2);
        lgd1 = legend([h1, h2, h3, h4, h5, h6, h7, h8, hb], {'AUV 1', 'AUV 2', 'AUV 3', 'AUV 4', 'AUV 5', 'AUV 6', 'AUV 7', 'AUV 8', 'Safety buffer'}, 'Location', 'northeast');
        lgd1.BoxFace.ColorType = 'truecoloralpha';
        lgd1.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
    case 3
        h1 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(1,:), 'MarkerEdgeColor', 'none');
        h2 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(2,:), 'MarkerEdgeColor', 'none');
        h3 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(3,:), 'MarkerEdgeColor', 'none');
        h4 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(4,:), 'MarkerEdgeColor', 'none');
        h5 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(5,:), 'MarkerEdgeColor', 'none');
        h6 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(6,:), 'MarkerEdgeColor', 'none');
        h7 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(7,:), 'MarkerEdgeColor', 'none');
        h8 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(8,:), 'MarkerEdgeColor', 'none');
        h9 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(9,:), 'MarkerEdgeColor', 'none');
        h10 = plot(NaN, NaN, 'o', 'MarkerSize', makerSize, 'MarkerFaceColor', Colors(10,:), 'MarkerEdgeColor', 'none');
        hb = scatter3(NaN, NaN, NaN, 80, [0 0 0], 'filled', 'MarkerFaceAlpha', 0.2, 'MarkerEdgeAlpha', 0.2);
        lgd1 = legend([h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, hb], {'AUV 1', 'AUV 2', 'AUV 3', 'AUV 4', 'AUV 5', 'AUV 6', 'AUV 7', 'AUV 8', 'AUV 9', 'AUV 10', 'Safety buffer'}, 'Location', 'northeast');
        lgd1.BoxFace.ColorType = 'truecoloralpha';
        lgd1.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
end

hold off;
xlim([0 100]);
ylim([0 100]);
zlim([-100,0])
set(gca,'FontName','Times New Roman','FontSize',12);
set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')


% 获取路径维度信息
[num_time_steps, ~, num_AUV] = size(path_Unif);
% 初始化存储距离矩阵
min_distances = zeros(num_time_steps, num_AUV); % 存储每个时刻每个 AUV 的最小距离
% 遍历每个时间点
for t = 1:num_time_steps
    % 当前时间点所有 AUV 的位置
    positions = squeeze(path_Unif(t, :, :))'; % 9 x 3 矩阵，每行是一个 AUV 的 (x, y, z)
    % 计算所有 AUV 两两之间的距离矩阵
    dist_matrix = squareform(pdist(positions)); % 计算 pairwise 距离 (9 x 9 矩阵)
    % 遍历每个 AUV，计算其与其他 AUV 的最小距离
    for i = 1:num_AUV
        % 排除自己，与其他 AUV 的距离
        other_distances = dist_matrix(i, :);
        other_distances(i) = Inf; % 将自身距离设为 Inf
        min_distances(t, i) = min(other_distances); % 记录最小距离
    end
end
% 绘制最小距离曲线
figure;
hold on;
sailing_time = linspace(0, colTime, num_time_steps);
for i = 1:num_AUV
    plot(sailing_time, min_distances(:, i), 'Color', Colors(i,:), 'DisplayName', sprintf('AUV %d', i));
end
% 给定两个点的坐标
point1 = [0, 0];
point2 = [colTime, safeDist];
% 提取两个点的坐标信息
x1 = point1(1);
y1 = point1(2);
x2 = point2(1);
y2 = point2(2);
% 计算矩形的宽度和高度
width = abs(x2 - x1);
height = abs(y2 - y1);
% 计算矩形的左下角坐标
x_left = min(x1, x2);
y_bottom = min(y1, y2);
% 绘制矩形并设置填充颜色和透明度
rectangle('Position', [x_left, y_bottom, width, height], 'FaceColor', [1, 0, 0, 0.2], 'EdgeColor', 'none');
hold off;

% 图形设置
xlabel('Time (s)');
ylabel('Minimum distance to other AUVs (m)');
% title(problem)
lgd1 = legend('show');
lgd1.BoxFace.ColorType = 'truecoloralpha';
lgd1.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
set(gca,'FontName','Times New Roman','FontSize',12);
set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')
xlim([0 colTime])
grid on; box on;


% ==== 新增：绘制俯仰角和偏航角随时间变化 ====
% 预分配
yaw   = zeros(num_time_steps, num_AUV);
pitch = zeros(num_time_steps, num_AUV);

% 逐 AUV、逐时间步计算
for k = 1:num_AUV
    for t = 1:num_time_steps-1
        delta = squeeze(path_Unif(t+1,:,k) - path_Unif(t,:,k));  % [dx,dy,dz]
        yaw(t,k)   = atan2(delta(2), delta(1));              % 偏航角 (rad)
        pitch(t,k) = atan2(delta(3), norm(delta(1:2)));      % 俯仰角 (rad)
    end
    % 最后一个点保持与倒数第二点一致
    yaw(end,k)   = yaw(end-1,k);
    pitch(end,k) = pitch(end-1,k);
end

% 转成度数，更直观
yaw   = rad2deg(yaw);
pitch = rad2deg(pitch);

% 绘制
figure('Position',[100,150,600,600]);
% 1）偏航角
subplot(2,1,1);
hold on; grid on; box on;
for k = 1:num_AUV
    plot(sailing_time, yaw(:,k), 'Color', Colors(k,:), 'DisplayName', sprintf('AUV %d', k));
end
xlabel('Time (s)');
ylabel('Yaw (°)');
lgd2 = legend('show');
lgd2.BoxFace.ColorType = 'truecoloralpha';
lgd2.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
set(gca,'FontName','Times New Roman','FontSize',12);
% set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')
xlim([0 colTime])
hold off;

% 2）俯仰角
subplot(2,1,2);
hold on; grid on; box on;
for k = 1:num_AUV
    plot(sailing_time, pitch(:,k), 'Color', Colors(k,:), 'DisplayName', sprintf('AUV %d', k));
end
xlabel('Time (s)');
ylabel('Pitch (°)');
lgd3 = legend('show');
lgd3.BoxFace.ColorType = 'truecoloralpha';
lgd3.BoxFace.ColorData(4) = 128; %设置Alpha值为128，取值范围0~255，越小越透明
set(gca,'FontName','Times New Roman','FontSize',12);
% set(gca,'LooseInset',[0.01 0.01 0.01 0.01])
set(gcf,'renderer','painters')
xlim([0 colTime])
hold off;
end

function plotSafetyBuffer(points, radius, colorRGB, skip)
% 在给定路径点处绘制安全缓冲球体
% points : Nx3 (x,y,z)
% radius : 球半径（安全距离）
% colorRGB : 1x3 颜色（与该 AUV 一致）
% skip : 下采样步长（每隔 skip 个点画一个球）

    if nargin < 4 || isempty(skip), skip = 10; end
    N = size(points,1);
    idx = 1:skip:N;
    if idx(end) ~= N, idx = [idx, N]; end  % 确保末端也画

    % 预生成一个单位球面网格，后续按半径缩放
    [sx, sy, sz] = sphere(16);    % 16 可调：越大球越光滑，开销越大
    alphaVal = 0.2;              % 透明度（建议很低，不挡视线）

    for t = idx
        cx = points(t,1); cy = points(t,2); cz = points(t,3);
        hx = sx*radius + cx;
        hy = sy*radius + cy;
        hz = sz*radius + cz;
        s = surf(hx, hy, hz, ...
                 'FaceColor', colorRGB, ...
                 'EdgeColor', 'none', ...
                 'FaceAlpha', alphaVal);
        % 为了避免遮挡感过强，可关闭背面剔除（默认即可），或适当减小 alpha
    end
end
