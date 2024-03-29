s = tf('s');
z = tf('z');
tr = 0.5;
ts = 1;
wn = 1.8/tr;
zeta = 4.6/(ts*wn);
dt = 0.04;
%% Design 1
% Ks = wn^2*s/(s^3+2*zeta*wn*s^2+wn^2*s-wn^2)
% Hs = 1/s;
% Gs = 1;
% Cs_w = minreal(Ks*Gs/(1+Gs*Ks*Hs))
%% Design 2
% Ks = s*wn^2/(s^2+2*zeta*wn*s)
% Ks = wn^2/(s+2*zeta*wn)
% Gs = 1;
% Gs2 = 1/s;
% Cs = minreal(Ks*Gs*Gs2/(1+Gs*Ks*Gs2))
% Ds = minreal(Gs*Gs2/(1+Gs*Gs2*Ks))
% Cz = c2d(Cs,dt,'zoh')
% Dz = c2d(Ds,dt,'zoh')
%% Design 3
Kp = 3;
Ki = 5;
Ks = Kp+Ki/s;
Gs = 1/s;
Cs = minreal(Gs/(1+Gs*Ks))
Kz = c2d(Cs,dt,'zoh')
%% Plots
close all;
figure;
impulseplot(Cs)
figure;
stepplot(Cs)
figure;
stepplot(Cs/s)
title('Ramp Response')
% figure;
% impulseplot(Ds)
% figure;
% stepplot(Ds)
% figure;
% impulseplot(Cz)
% figure;
% stepplot(Cz)