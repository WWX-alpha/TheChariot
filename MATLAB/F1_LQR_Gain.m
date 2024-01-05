%% 输入：系统矩阵A ， B；权重矩阵 Q、R、S;
%% 输出： 反馈增益矩阵F；
function [F] = F1_LQR_Gain(A , B , Q , R , S)

%计算系统矩阵维度n;
n = size(A , 1);
%计算输入矩阵维度p;
p = size(B , 2);
%系统终值代价权重矩阵，定义为P0;
P0 = S;
%定义最大迭代次数，用于限制程序运行的时间；
max_iter = 200;
%初始化矩阵P为0矩阵，后续用于存放计算得到的一系列矩阵P[k]
P = zeros(n , n * max_iter);
%初始化矩阵P的第一个位置为P0；
P(: , 1:n) = P0;
%定义P[k - 1]的初值为P0，即当k = 1时；
P_k_min_1 = P0;
%定义系统稳态误差阈值，用于判断系统是否达到稳态；
tol = 1e-3;
%初始化系统的误差为无穷；
diff = Inf;
%初始系统的反馈增益为无穷；
F_N_min_k = inf;
%初始化系统的迭代步数；
k = 1;

%判断系统是否达到稳态，即相邻的增益误差是否小于预设误差，如达到稳态跳出while循环
while diff > tol
%将系统增益F[N - K]赋值给Fpre[N - K].此步骤用于判断系统是否达到稳态
F_N_min_k_pre = F_N_min_k;
%计算F[N - K]
F_N_min_k = inv(R + B' * P_k_min_1 * B) * B' * P_k_min_1 * A;
%计算P[K]
P_k = (A - B * F_N_min_k)' * P_k_min_1 * (A - B * F_N_min_k) + (F_N_min_k)' * R * (F_N_min_k) + Q;
%将P[K]j矩阵存入P矩阵的相应位置
P(: , n * k - n + 1 : n * k) = P_k;
%更新P[K - 1], 用于下一次的迭代
P_k_min_1 = P_k;
%计算系统相邻步增益差值
diff = abs(max(F_N_min_k - F_N_min_k_pre));
%迭代步加1
k = k + 1;
%如程序超过预设最大迭代步，则报错
if k > max_iter
    error('Maximum Number of Iterations Exceeded');
end
end
%输出系统跌代步
fprintf('No.of Interation is %d \n' , k);
%模块输出：系统增益F
F = F_N_min_k;
end

