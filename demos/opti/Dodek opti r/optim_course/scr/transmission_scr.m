%
% Optimalny navrh prevodoveho pomeru elektrickeho pohonu
%

clear;
clc;
close all;


%% ============================================================
% PARAMETRE POHONU
% =============================================================

r          = 0.30;       % polomer kolesa [m]
omega_w    = 30;         % uhlova rychlost kolesa [rad/s]
F_req      = 1200;       % pozadovana trakcna sila [N]

T_max      = 200;        % maximalny moment motora [Nm]

i_min      = 2;          % minimalny prevod
i_max      = 8;          % maximalny prevod

% Parametre ucinnosti prevodovky
eta0       = 0.95;
i0         = 5;
a          = 0.04;
b          = 1e-5;

% Penalizacia momentu motora
alpha      = 0.05;

% Penalizacia momentu motora
alpha      = 0.05;


%% ============================================================
% OPTIMALIZACNE PREMENNE A OHRANICENIA
% =============================================================


%lb = %TODO
%ub = %TODO


% ------------------------------------------------------------
% Ucelova funkcia
% ------------------------------------------------------------

objective = @(x) gearbox_objective( ...
    x,omega_w,eta0,i0,a,b,alpha);


% ------------------------------------------------------------
% Nelinearne obmedzenia
% ------------------------------------------------------------

nonlcon = @(x) gearbox_constraints( ...
    x,r,F_req,eta0,i0,a,b);


%% ============================================================
% NASTAVENIE fmincon
% =============================================================

% options = optimoptions(%TODO);


%% ============================================================
% SPUSTENIE fmincon
% =============================================================

% Pociatocny bod
x0 = [
    5;
    100
];



% [x_opt,J_opt,exitflag,output] = fmincon( %TODO);

%% ============================================================
% VYSLEDOK
% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('VYSLEDOK OPTIMALIZACIE\n');
fprintf('=============================================\n');

fprintf('Prevodovy pomer i       = %.8f\n',x_opt(1));
fprintf('Moment motora Tm        = %.8f Nm\n',x_opt(2));
fprintf('Ucelova funkcia J       = %.8f\n',J_opt);

fprintf('\n');


% Vypocet dalsich velicin

i  = x_opt(1);
Tm = x_opt(2);

eta = eta0 ...
    - a*(i-i0)^2 ...
    - b*Tm^2;

omega_m = i*omega_w;

Tw = eta*i*Tm;

F = Tw/r;

P_loss = (1-eta)*i*Tm*omega_w;


fprintf('Ucinnost eta            = %.8f\n',eta);
fprintf('Otacky motora omega_m   = %.8f rad/s\n',omega_m);
fprintf('Moment na kolese Tw     = %.8f Nm\n',Tw);
fprintf('Trakcna sila F          = %.8f N\n',F);
fprintf('Straty P_loss           = %.8f W\n',P_loss);

fprintf('\n');

fprintf('exitflag                = %d\n',exitflag);
fprintf('Pocet iteracii          = %d\n',output.iterations);

fprintf('=============================================\n');



%% ============================================================
% GRAFICKA VIZUALIZACIA
% =============================================================

figure('Color','w');

hold on;
grid on;
box on;


%% ------------------------------------------------------------
% Mriezka v rovine (i,Tm)
% ------------------------------------------------------------

i_plot = linspace(i_min,i_max,400);
T_plot = linspace(0,T_max,400);

[I,T] = meshgrid(i_plot,T_plot);


%% ------------------------------------------------------------
% Ucinnost
% ------------------------------------------------------------

ETA = eta0 ...
    - a*(I-i0).^2 ...
    - b*T.^2;


%% ------------------------------------------------------------
% Trakcna sila
% ------------------------------------------------------------

F_grid = ETA .* I .* T ./ r;


%% ------------------------------------------------------------
% Ucelova funkcia
% ------------------------------------------------------------

J_grid = (1-ETA).*I.*T.*omega_w ...
    + alpha*T.^2;


%% ============================================================
% VRSTEVNICE UCELOVEJ FUNKCIE
% =============================================================

contour( ...
    I,T,J_grid,100, ...
    'LineWidth',0.8);


%% ============================================================
% HRANICA POZADOVANEJ TRAKCNEJ SILY
% =============================================================

contour( ...
    I,T,F_grid,[F_req F_req], ...
    'LineWidth',2);


%% ============================================================
% PRIPUSTNA OBLAST F >= F_req
% =============================================================

F_feasible = F_grid;

F_feasible(F_feasible < F_req) = NaN;

contourf( ...
    I,T,F_feasible, ...
    [F_req max(F_grid(:))], ...
    'FaceAlpha',0.12, ...
    'LineStyle','none');




%% ============================================================
% HRANICA MAXIMALNEHO MOMENTU
% =============================================================

hTorque = plot( ...
    [i_min i_max], ...
    [T_max T_max], ...
    '-r', ...
    'LineWidth',2);


%% ============================================================
% BOX CONSTRAINTS
% =============================================================

% i = i_min
hImin = plot( ...
    [i_min i_min], ...
    [0 T_max], ...
    '-r', ...
    'LineWidth',1.5);


% i = i_max
hImax = plot( ...
    [i_max i_max], ...
    [0 T_max], ...
    '-r', ...
    'LineWidth',1.5);


% Tm = 0
hTmin = plot( ...
    [i_min i_max], ...
    [0 0], ...
    '-r', ...
    'LineWidth',1.5);


%% ============================================================
% POCIATOCNY BOD
% =============================================================

hStart = plot( ...
    x0(1),x0(2), ...
    'o', ...
    'MarkerSize',8, ...
    'LineWidth',2);


%% ============================================================
% OPTIMALNE RIESENIE
% =============================================================

hOpt = plot( ...
    x_opt(1),x_opt(2), ...
    'x', ...
    'MarkerSize',15, ...
    'LineWidth',2);


%% ============================================================
% POPIS GRAFU
% =============================================================

xlabel('Prevodový pomer i');
ylabel('Moment elektromotora T_m [Nm]');

title('Optimalizácia prevodového pomeru');

xlim([i_min i_max]);
ylim([0 T_max]);




%% ============================================================
% LOKALNE FUNKCIE
% =============================================================


function [J,dJ] = gearbox_objective(x,omega_w,eta0,i0,a,b,alpha)

   %TODO

end


function [c,ceq,GC,GCeq] = gearbox_constraints(x,r,F_req,eta0,i0,a,b)

  %TODO

end