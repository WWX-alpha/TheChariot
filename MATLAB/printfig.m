%%
close all;
clc;
%%
figure();
hold on;
grid on;
t = 1:1:31;
plot(in, out, 'linewidth', 1);
%plot(t, out, 'linewidth', 1);

axis([0 350 0 500]);
%legend('Ŀ��ת��','ʵ��ת��');
%legend('Location', 'northeast');
xlabel('���(pixel)');
ylabel('����(m)');