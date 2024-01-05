%%
clear all;
close all;
clc;
%%
Gs = tf([0.20566],[0.47113 1]);

figure();
hold on;
grid on;
bode(Gs);
margin(Gs);

figure();
hold on;
grid on;
rlocus(Gs);

[A,B,C,D] = tf2ss([0.20566],[0.47113 1]);