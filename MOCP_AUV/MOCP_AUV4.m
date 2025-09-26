classdef MOCP_AUV4 < PROBLEM
% <2025> <multi> <real/binary> <large/constrained/none>
% Multi-objective Collaborative Path Planning for Autonomous Underwater
% Vehicle (AUV)

    properties(Access = private)
        AUV; % AUV信息
        ENV; % 环境信息
    end
    methods
        %% Default settings of the problem
        function Setting(obj)
            % Parameter setting
            obj.AUV.num = 8; % AUV的数量
            obj.AUV.cruise = [2.31, 2.35, 2.16, 2.27, 2.09, 2.43, 2.45, 2.02]; % 最大巡航速度
            obj.AUV.initialBattery = [13.37, 14.49, 14.87, 14.60, 14.66, 14.69, 14.39, 13.34]; % 初始可用能量12-15 kJ
            obj.AUV.startPos = [0 50 -50; 0 0 -50; 50 0 -50; 100 0 -50; 100 50 -50; 100 100 -50; 50 100 -50; 0 100 -50]; %%% 起始点
            obj.AUV.goalPos = [100 50 -50; 100 100 -50; 50 100 -50; 0 100 -50; 0 50 -50; 0 0 -50; 50 0 -50; 100 0 -50]; %%% 目标点
            
            obj.AUV.searchRange = [0, 100; 0, 100; -100, 0]; % 搜索x、y、z范围
            obj.AUV.P_cruise = 0.08; % 巡航的单位能耗kW
            obj.AUV.P_turn = 0.03;   % 转向引起的额外能耗，假设满转90度相当于消耗额外0.03 kW
            obj.AUV.P_climb = 0.02;  % 爬升引起的额外能耗，假设爬升1米相当于消耗额外0.02 kW
            obj.AUV.density = 100; % 插值密度
            obj.AUV.safeDist = 3; % 最小安全距离

            obj.ENV.dim = 3; % 3维路径
            % 定义多个涡流源的参数（Lamb–Oseen 涡流模型[x0, y0, z0, λ, ζ]）
            %   x0, y0, z0 为涡心坐标（整数）
            %   λ > 0 逆时针；λ < 0 顺时针
            %   ζ 控制涡核尺度
            %  |λ| 越大，涡周向速度越大，流场更"猛"; λ ≈ 6.3 ζ U0.
            %   ζ 越小，流速集中在涡心附近，核心更"尖锐"; ζ 越大，流场扩散更开，整个涡影响半径更大; 常选 ζ∈[0.1L,0.3L]
            obj.ENV.vortices = [
                25, 50, -25, -63, 10;   % 涡1: 涡心(25,50,-25)，λ≈6.3·ζ·U0，U0取1
                50, 75, -40,  95, 15;   % 涡2: 涡心(50,75,-40)
                75, 25, -60, -75, 12    % 涡3: 涡心(75,25,-60)
                ];

            % === 时变洋流参数（可调） ===
            % 强度随时间起伏：lambda_t = lambda0 * (1 + A_lambda * sin(omega_lambda * t + phi_lambda))
            % 涡心小幅漂移：  c_t = c0 + A_shift .* sin(omega_shift * t + phi_shift)
            % 随机噪声（加到速度分量上）：N(0, noise_std^2)
            obj.ENV.currTemporal.A_lambda   = 0.30;           % 强度起伏幅度（0~0.5常用）
            obj.ENV.currTemporal.omega_lam  = 2*pi/600;       % 强度起伏角频率（单位：1/s，按仿真时间标度设置）
            obj.ENV.currTemporal.phi_lam    = 0.0;            % 强度相位
            obj.ENV.currTemporal.A_shift    = [2, 2, 1];      % 涡心漂移幅度（单位与坐标一致）
            obj.ENV.currTemporal.omega_shift= 2*pi/900;       % 漂移角频率
            obj.ENV.currTemporal.phi_shift  = [0, pi/3, pi/6];% 漂移相位
            obj.ENV.currTemporal.noise_std  = 0.02;           % 速度噪声标准差（m/s 量级，自行调参）

            % 威胁区域矩阵 [x_c, y_c, z_c, e, f, g, m, n, q]
            Threats = [
                19, 22, -34, 9.00, 9.00, 9.00, 1.00, 1.00, 1.00
                19, 25, -82, 5.17, 9.24, 9.66, 1.51, 1.63, 1.61
                20, 82, -84, 10.00, 10.00, 10.00, 1.00, 1.00, 1.00
                22, 50, -58, 10.00, 12.00, 40.00, 0.50, 0.50, 1.00
                25, 80, -37, 9.78, 7.42, 25.00, 0.71, 1.13, 1.87
                45, 45, -49, 5.54, 28.77, 10.38, 1.51, 1.48, 1.74
                55, 58, -73, 15.58, 5.42, 6.78, 0.93, 0.97, 2.35
                69, 32, -84, 13.00, 13.00, 13.00, 1.00, 1.00, 1.00
                69, 89, -46, 17.73, 5.69, 10.74, 0.88, 1.76, 0.88
                79, 66, -33, 12.53, 12.15, 28.38, 1.06, 1.04, 0.22
                80, 24, -41, 10.74, 7.98, 16.25, 0.80, 0.80, 1.00
                81, 80, -83, 4.57, 12.16, 7.49, 1.21, 1.20, 1.02
                ];
            obj.ENV.threat = Threats;

            Nodes = 10; % 导航节点数

            obj.M        = 3;
            obj.D        = obj.ENV.dim * Nodes * obj.AUV.num;
            lower = [];
            upper = [];
            encoding = [];
            for i = 1:obj.AUV.num
                d = pdist([obj.AUV.startPos(i,:); obj.AUV.goalPos(i,:)]);
                r = d / (Nodes+1);
                scale = [1:1:Nodes/2, Nodes/2:-1:1];
                lower = [lower, zeros(1,Nodes), -r*scale, -r*scale];
                upper = [upper, ones(1,Nodes), r*scale, r*scale];
                encoding = [encoding, 4+zeros(1,Nodes), 1+zeros(1,2*Nodes)];
            end
            obj.lower    = lower;
            obj.upper    = upper;
            obj.encoding = encoding; % 混合编码
        end
        %% Evaluate objective values
        function Population = Evaluation(obj,varargin)
            PopDec = varargin{1};
            Pop = PopDec;
            AUV_num = obj.AUV.num; % USV的数量
            cruise = obj.AUV.cruise;
            initialBattery = obj.AUV.initialBattery;
            P_cruise = obj.AUV.P_cruise;
            P_turn = obj.AUV.P_turn;
            P_climb = obj.AUV.P_climb;
            Density = obj.AUV.density; % 插值密度
            safeDist = obj.AUV.safeDist;
            ThreatArea = obj.ENV.threat;

            Fitness = zeros(size(Pop, 1), 3); % 3个目标值
            Constraint = zeros(size(Pop, 1), 5*AUV_num + 1); %%%%%%%%%%%%%%%%%%%%%

            temp_AUV = obj.AUV; % 为了减少通信开销的变量
            dim = obj.ENV.dim;
            % 获取环境中涡流参数
            vortices = obj.ENV.vortices;     % 涡流强度数组
            % 定义影响因子，需根据实际情况调参
            k_w = 0.3; % 水流影响因子
            range = obj.AUV.searchRange;
            for m = 1:size(Pop, 1)
                Agent = Pop(m,:);
                multi_Path = decodePath(temp_AUV, Agent); % 将搜索代理解码为航行路径
                segment_Distance = cell(AUV_num, 1); % 存储每个 AUV 路径段的距离
                time_Intervals = cell(AUV_num, 1); % 存储每个 AUV 路径段的时间步长
                path_Unif = zeros(Density, dim, AUV_num); % 存储每个 AUV 的等时间间隔路径点
                
                %%% 单个USV的路径约束
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

                    % === 估算每个路径节点的时间戳 t1（先用名义巡航速度近似）===
                    %   用当前AUV的最大巡航速度作为名义速度，按弧长积分得到时间
                    seg_dt_nominal = segment_Distance{k} ./ max(cruise(k), eps);   % 每段名义用时
                    t1 = [0, cumsum(seg_dt_nominal)];                              % 与 x1/y1/z1 对齐的时间序列（秒）

                    % === 提取“时变洋流”速度场 ===
                    waterFlow_x = zeros(size(x1));
                    waterFlow_y = zeros(size(y1));
                    waterFlow_z = zeros(size(z1));

                    currP = obj.ENV.currTemporal;   % 时变参数
                    % 计算涡流模型的水流速度
                    for j = 1:size(vortices,1)
                        % 基础涡参数
                        x0     = vortices(j,1);
                        y0     = vortices(j,2);
                        z0     = vortices(j,3);
                        lambda0= vortices(j,4);
                        zeta   = vortices(j,5);

                        % === 随时间变化的涡心位置（小幅漂移）===
                        x0_t = x0 + currP.A_shift(1) * sin(currP.omega_shift * t1 + currP.phi_shift(1));
                        y0_t = y0 + currP.A_shift(2) * sin(currP.omega_shift * t1 + currP.phi_shift(2));
                        z0_t = z0 + currP.A_shift(3) * sin(currP.omega_shift * t1 + currP.phi_shift(3));

                        % === 随时间变化的涡强 ===
                        lambda_t = lambda0 .* (1 + currP.A_lambda * sin(currP.omega_lam * t1 + currP.phi_lam));

                        % 对每个节点计算速度分量
                        dx = (x1 - x0_t);  dy = (y1 - y0_t);  dz = (z1 - z0_t);
                        r2 = dx.^2 + dy.^2 + dz.^2 + eps;          % 避免除零
                        factor = 1 - exp(-r2 ./ (zeta^2));

                        % Lamb–Oseen 的周向分量：平面内构成旋转流 + 垂向分量（与原来一致）
                        u = -lambda_t .* dy ./ (2*pi*r2) .* factor;      % x 方向
                        v =  lambda_t .* dx ./ (2*pi*r2) .* factor;      % y 方向
                        w =  lambda_t ./ (pi * zeta^2) .* exp(-r2./(zeta^2));  % z 方向（可按需要保留/缩放）

                        % 累加多个涡流的影响
                        waterFlow_x = waterFlow_x + u;
                        waterFlow_y = waterFlow_y + v;
                        waterFlow_z = waterFlow_z + w;
                    end

                    % 可选：给速度叠加小噪声，模拟测量/建模误差
                    if obj.ENV.currTemporal.noise_std > 0
                        waterFlow_x = waterFlow_x + obj.ENV.currTemporal.noise_std * randn(size(waterFlow_x));
                        waterFlow_y = waterFlow_y + obj.ENV.currTemporal.noise_std * randn(size(waterFlow_y));
                        waterFlow_z = waterFlow_z + obj.ENV.currTemporal.noise_std * randn(size(waterFlow_z));
                    end

                    % 将时变水流速度赋值给环境对象
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
                    % 计算出第 k 个AUV的协同速度，对巡航速度进行比例缩放
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
                    % 若D_min >= 1，则该点不处于任何威胁区域内部，该点威胁代价为1/(1+(D_min-1)^2)。
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

            % Objective function
            PopObj = Fitness;
            % Constraints
            PopCon = Constraint;
            
            Population = SOLUTION(PopDec,PopObj,PopCon,varargin{2:end});
            obj.FE     = obj.FE + length(Population);
        end
        %% 定义一个公共方法，用于获取案例信息
        function [AUV, ENV] = getCaseInfo(obj)
            AUV = obj.AUV; % AUV信息
            ENV = obj.ENV; % 环境信息
        end

        %% Generate a point for hypervolume calculation
        function R = GetOptimum(obj,~)
            R = [108.52, 300.98, 74.71] * 1.1; % 参考点
        end
    end
end

%% 路径解码函数
function multi_Path = decodePath(AUV, Agent)
% decodePath 将搜索代理解码成世界坐标系中的路径点
%
% 输入参数：
% Agent 搜索代理
% Density 插值密度
% startPos 路径起点
% goalPos 路径终点

    AUV_num = AUV.num;
    Density = AUV.density; % 插值密度
    startPos = AUV.startPos;
    goalPos = AUV.goalPos;

    AgentR = reshape(Agent, size(Agent,2) ./ AUV_num, AUV_num)';
    multi_Path = cell(AUV_num, 1);
    for i = 1:AUV_num
        AgentI = AgentR(i,:);
        AgentM = reshape(AgentI, size(AgentI,2) ./ 3, 3)'; % 使用 reshape 函数将行向量重新构造为USV_num * (3*Nodes)的数组
        AgentM(1,:) = round(AgentM(1,:)); % 对第一行变量取整
        axisX = sqrt((goalPos(i,1)-startPos(i,1))^2 + (goalPos(i,2)-startPos(i,2))^2 + (goalPos(i,3)-startPos(i,3))^2); % 距离在X轴上的投影
        Xh = linspace(0, axisX, size(AgentM,2)+2); % 在齐次坐标系中的X坐标
        Path = [1,AgentM(1,:),1; Xh; 0,AgentM(2,:),0; 0,AgentM(3,:),0];
        Valid_path = Path(2:end, Path(1,:)>0); % 组建有效的路径点
        % 路径插值，分段三次 Hermite 插值多项式
        Xh = Valid_path(1,:);  Yh = Valid_path(2,:);  Zh = Valid_path(3,:);

        dX = diff(Xh);  dY = diff(Yh);  dZ = diff(Zh);
        distances = sqrt(dX.^2 + dY.^2 + dZ.^2);

        t_param = [0, cumsum(distances)]; % 弧长参数 t，从0开始

        t_fine = linspace(t_param(1), t_param(end), Density);
        % 分别对X(t), Y(t), Z(t)进行插值
        xh = interp1(t_param, Xh, t_fine, 'pchip');
        yh = interp1(t_param, Yh, t_fine, 'pchip');
        zh = interp1(t_param, Zh, t_fine, 'pchip');

        X_B = [xh;yh;zh]; % 齐次坐标系中的坐标点
        PathM = transformBtoA(startPos(i,:), goalPos(i,:), X_B); % 转换为世界坐标系中的坐标点
        multi_Path{i} = PathM;
    end

end

%% 齐次坐标变换函数
function X_A = transformBtoA(p, q, X_B)
% transformBtoA 将坐标系B中的点转换到坐标系A
%
% 输入参数：
%   p   : 1x3向量，坐标系A中定义的点p的位置
%   q   : 1x3向量，坐标系A中定义的点q的位置
%   X_B : 3xN矩阵，在坐标系B中的点坐标，每列一个点 [x_b, y_b, z_b]
%
% 输出参数：
%   X_A : 3xN矩阵，对应点在A坐标系下的坐标表示
%
% 原理：
% 定义向量 v = q - p。
% α = atan2(v_y, v_x)，通过绕Z轴旋转 -α 使v投影对齐X轴。
% 然后在该中间坐标系下计算 β = atan2(v'_z, v'_x)，通过绕Y轴 -β 使v对齐X轴。
% 因此B到A的变换为：X_A = p + R_z(α)*R_y(β)*X_B
% 使用齐次坐标变换矩阵统一处理多个点。
%
% 调用案例
% 已知点p和q在A坐标系下的坐标
% p = [2 2 3];
% q = [10 5 5];
% X_B = [2, 3, 5;5, 8, 10;-5, -3, 2]';
% X_A = transformBtoA(p, q, X_B);

    v = q - p;
    
    % 计算α角度（范围在[-pi,pi]）：v的XOY平面投影与X轴夹角
    alpha = atan2(v(2), v(1));
    
    % 绕Z轴(-α)旋转后v在此坐标系下为v_Aprime
    Rz_minus_alpha = [cos(-alpha) -sin(-alpha) 0;
                      sin(-alpha)  cos(-alpha) 0;
                      0            0           1];
    v_Aprime = Rz_minus_alpha * v';
    
    % 计算β角度：v_Aprime相对于X轴在XZ平面的仰角
    beta = atan2(v_Aprime(3), v_Aprime(1));
    
    % 绕Y轴 -β，可将v对齐X轴
    % Ry_minus_beta = [cos(-beta), 0, -sin(-beta);
    %                  0,          1, 0;
    %                  sin(-beta),0, cos(-beta)];
    
    % B到A的旋转矩阵为 R_B2A = Rz(α)*Ry(β)
    % 但是我们已知 A到B的R是Ry(-β)*Rz(-α)，所以B到A为其逆：R_B2A = Rz(α)*Ry(β)
    % 直接构造R_B2A
    
    Rz_alpha = [cos(alpha) -sin(alpha) 0;
                sin(alpha)  cos(alpha) 0;
                0           0          1];
    
    Ry_beta = [cos(beta) 0 -sin(beta);
               0         1 0;
               sin(beta) 0 cos(beta)];
    
    
    R_B2A = Rz_alpha * Ry_beta;
    
    % B原点在A中的位置为p，所以齐次变换从B到A为：
    % X_A = p + R_B2A * X_B
    % 将X_B点集转化为4xN的齐次坐标
    N = size(X_B,2);
    X_B_hom = [X_B; ones(1,N)]; % 4xN
    
    
    % 构造B到A的齐次变换矩阵
    T_B2A = [R_B2A, p'; 0 0 0 1];
    
    % 应用变换
    X_A_hom = T_B2A * X_B_hom; % 4xN
    
    % 提取前三行为A系坐标
    X_A = X_A_hom(1:3,:);

end
