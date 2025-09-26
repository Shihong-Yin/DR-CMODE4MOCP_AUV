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