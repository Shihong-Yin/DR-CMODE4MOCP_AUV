function insideMask = pointsInsideThreats(points, threats)
% pointsInsideThreats - 对一批点判断它们是否在任一威胁区域内部
%
% 输入参数：
%   points    : N x 3 矩阵，每行代表一个点的坐标 [X, Y, Z]
%   obstacles : 结构数组，每个元素包含字段 x_c, y_c, z_c, e, f, g, m, n, q
%
% 输出参数：
%   insideMask: N x 1 的逻辑数组，对应每个点是否在任意一个威胁区域内部

    N = size(points,1);
    insideMask = false(N,1);

    % 将点的坐标分解
    X = points(:,1);
    Y = points(:,2);
    Z = points(:,3);

    numThreats = size(threats, 1);

    % 对于每个威胁区域逐一判断
    for i = 1:numThreats
        threat = threats(i, :); % 取出第 i 个威胁区域
        % 计算 F 值
        u = (X - threat(1))/threat(4);
        v = (Y - threat(2))/threat(5);
        w = (Z - threat(3))/threat(6);

        Fvals = (abs(u).^(2*threat(7))) + (abs(v).^(2*threat(8))) + (abs(w).^(2*threat(9)));

        % 判断哪些点在该障碍物内部
        % 内部：F < 1
        insideMask = insideMask | (Fvals < 1);
    end

end
