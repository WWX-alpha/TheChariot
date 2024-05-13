%%
close all;
clc;
%%
figure();
hold on;
grid on;

plot(in, out, 'linewidth', 1);
%plot(t, out, 'linewidth', 1);
plot(in, 1.971e4 .* in.^-0.9089, 'linewidth', 1);

axis([0 350 0 500]);
legend('����ֵ','���ֵ');
legend('Location', 'northeast');
xlabel('���(pixel)');
ylabel('����(m)');