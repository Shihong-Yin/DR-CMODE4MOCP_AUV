function isND = find_non_dominated(objectives, tol)
% 返回“非支配解”的目标值矩阵（按最小化问题定义）
% 支配定义：j 支配 i  <=>  (Fj <= Fi 且 至少一维 Fj < Fi)
% 增加容差 tol 以避免浮点误判；默认 1e-12
%
% inputs:
%   objectives : N×M 目标值矩阵
%   tol        : 数值容差，可选（默认 1e-12）
%
% output:
%   isND : 非支配解的索引

if nargin < 2
    tol = 1e-12;
end

[N, ~] = size(objectives);
isND = true(N,1);

for i = 1:N
    if ~isND(i), continue; end   % 已判为被支配可跳过
    Fi = objectives(i,:);
    for j = 1:N
        if j == i, continue; end
        Fj = objectives(j,:);

        % --- 标准支配判定（最小化）：all(Fj <= Fi) & any(Fj < Fi) ---
        if all(Fj <= Fi + tol) && any(Fj < Fi - tol)
            isND(i) = false;     % i 被 j 支配
            break;
        end
    end
end

end
