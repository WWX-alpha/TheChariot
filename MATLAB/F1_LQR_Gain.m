%% ���룺ϵͳ����A �� B��Ȩ�ؾ��� Q��R��S;
%% ����� �����������F��
function [F] = F1_LQR_Gain(A , B , Q , R , S)

%����ϵͳ����ά��n;
n = size(A , 1);
%�����������ά��p;
p = size(B , 2);
%ϵͳ��ֵ����Ȩ�ؾ��󣬶���ΪP0;
P0 = S;
%�����������������������Ƴ������е�ʱ�䣻
max_iter = 200;
%��ʼ������PΪ0���󣬺������ڴ�ż���õ���һϵ�о���P[k]
P = zeros(n , n * max_iter);
%��ʼ������P�ĵ�һ��λ��ΪP0��
P(: , 1:n) = P0;
%����P[k - 1]�ĳ�ֵΪP0������k = 1ʱ��
P_k_min_1 = P0;
%����ϵͳ��̬�����ֵ�������ж�ϵͳ�Ƿ�ﵽ��̬��
tol = 1e-3;
%��ʼ��ϵͳ�����Ϊ���
diff = Inf;
%��ʼϵͳ�ķ�������Ϊ���
F_N_min_k = inf;
%��ʼ��ϵͳ�ĵ���������
k = 1;

%�ж�ϵͳ�Ƿ�ﵽ��̬�������ڵ���������Ƿ�С��Ԥ������ﵽ��̬����whileѭ��
while diff > tol
%��ϵͳ����F[N - K]��ֵ��Fpre[N - K].�˲��������ж�ϵͳ�Ƿ�ﵽ��̬
F_N_min_k_pre = F_N_min_k;
%����F[N - K]
F_N_min_k = inv(R + B' * P_k_min_1 * B) * B' * P_k_min_1 * A;
%����P[K]
P_k = (A - B * F_N_min_k)' * P_k_min_1 * (A - B * F_N_min_k) + (F_N_min_k)' * R * (F_N_min_k) + Q;
%��P[K]j�������P�������Ӧλ��
P(: , n * k - n + 1 : n * k) = P_k;
%����P[K - 1], ������һ�εĵ���
P_k_min_1 = P_k;
%����ϵͳ���ڲ������ֵ
diff = abs(max(F_N_min_k - F_N_min_k_pre));
%��������1
k = k + 1;
%����򳬹�Ԥ�������������򱨴�
if k > max_iter
    error('Maximum Number of Iterations Exceeded');
end
end
%���ϵͳ������
fprintf('No.of Interation is %d \n' , k);
%ģ�������ϵͳ����F
F = F_N_min_k;
end

