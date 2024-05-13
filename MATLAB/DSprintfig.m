%%
close all;
clc;
%%
figure();
hold on;
grid on;

plot(speed, deg20, 'linewidth', 1);
plot(speed, deg30, 'linewidth', 1);
plot(speed, deg40, 'linewidth', 1);
plot(speed, deg50, 'linewidth', 1);

axis([2000 3600 0 300]);
legend('仰角20度','仰角30度','仰角40度','仰角45度');
legend('Location', 'southeast');
xlabel('转速(RPM)');
ylabel('距离(m)');
%%
figure();
hold on;
grid on;
plot(deg30, speed, 'linewidth', 1);
t = 0:1:300;
plot(t,-0.01613 .* t.^2  + 14.48 .* t + 805.2, 'linewidth', 1);
axis([0 300 0 3600]);
legend('测量值','拟合值');
legend('Location', 'southeast');
ylabel('转速(RPM)');
xlabel('距离(m)');