%%
clear all;
close all;
clc;
%%
figure();

res = 64;
for theta = [pi / 3, pi/4]
    L = 1;

    V = 1;

    A = [-1/sin(theta), 1/cos(theta) L;
            -1/sin(theta), -1/cos(theta) L;
            1/sin(theta), -1/cos(theta) L;
            1/sin(theta), 1/cos(theta) L;];

    R = [-cos(theta), -cos(theta), cos(theta), cos(theta);
        sin(theta), -sin(theta), -sin(theta), sin(theta)];

        for alpha = 0:pi/res:2*pi
            B = [cos(alpha),0;
                sin(alpha), 0;
                0, 1];
            Vonwheel = A * B * [V;0];
            [vmax,vindex] = max(abs(Vonwheel));
            K = 1/abs(vmax);
            Vonwheelnorm = Vonwheel * K;

            Vabs = R * Vonwheelnorm;
            Vx = Vabs(1,1)
            Vy = Vabs(2,1)

            Vnorm = Vx * cos(alpha) + Vy * sin(alpha);
            Vnorm = Vnorm ./ 4;
            if(alpha == 0)
               Vout = [alpha;
                   Vnorm] ;
            else
                Vout = [Vout [alpha;Vnorm]];
            end
        end
    polarplot(Vout(1,:), Vout(2,:), '-', 'linewidth', 1);
    hold on;
end

polarplot(Vout(1,:), ones(1,129), '-', 'linewidth', 1);
hold on;

grid on;
legend('60度全向底盘','45度全向底盘','4舵轮底盘');
legend('Location', 'northeast');