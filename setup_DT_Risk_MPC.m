clear; clc; close all;

Ts = 0.1;
Tsim = 60;

Np = 20;
Nc = 10;

vmax = 1.0;
vmin = 0.0;

wmax = 1.0;
wmin = -1.0;

d_safe = 0.5;
sigma = 1.0;
lambda_risk = 5.0;

scenario = 1;
% 1 = obstacle-free
% 2 = static obstacle
% 3 = dynamic obstacle