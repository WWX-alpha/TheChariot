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
%legend('目标转速','实际转速');
%legend('Location', 'northeast');
xlabel('宽度(pixel)');
ylabel('距离(m)');