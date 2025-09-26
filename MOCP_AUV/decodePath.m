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