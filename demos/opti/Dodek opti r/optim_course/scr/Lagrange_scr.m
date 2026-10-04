
clear;
clc;
close all;

%% 1. Definicia funkcii

%J = @(x,y) %TODO

%h = @(x,y) %TODO

% Gradient ucelovej funkcie
%gradJ = @(x,y) [%TODO];

% Gradient ohranicenia
%gradh = @(x,y) [%TODO];

%% 2. Extremalne body

%maxPoints = [ %TODO];

%minPoints = [ %TODO;
              %TODO];

%maxLambda = [%TODO];
%minLambda = [%TODO];

%% 3. Hodnoty ucelovej funkcie

maxValues = zeros(2,1);
minValues = zeros(2,1);

for i = 1:2
    maxValues(i) = J(maxPoints(i,1),maxPoints(i,2));
    minValues(i) = J(minPoints(i,1),minPoints(i,2));
end

%% 4. Mriezka pre vykreslenie plochy

x = linspace(-2.5,2.5,200);
y = linspace(-2.5,2.5,200);

[X,Y] = meshgrid(x,y);

Z = J(X,Y);

%% 5. Ohranicenie - kruznica

theta = linspace(0,2*pi,500);

xc = 2*cos(theta);
yc = 2*sin(theta);
zc = J(xc,yc);

%% 6. Hlavna 3D vizualizacia

figure( ...
    'Name',' ', ...
    'Color','w', ...
    'Position',[100 100 600 450]);

%% Ucelova funkcia

surf(X,Y,Z, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.72);

hold on;

%% Ohranicenie

plot3(xc,yc,zc, ...
    'k-', ...
    'LineWidth',2);

%% Extremalne body

plot3(maxPoints(:,1), ...
      maxPoints(:,2), ...
      maxValues, ...
      'ro', ...
      'MarkerSize',10, ...
      'MarkerFaceColor','r');

plot3(minPoints(:,1), ...
      minPoints(:,2), ...
      minValues, ...
      'bo', ...
      'MarkerSize',10, ...
      'MarkerFaceColor','b');


gradientScale = 0.35;

%% --- MAXIMA ---

for i = 1:size(maxPoints,1)

    x0 = maxPoints(i,1);
    y0 = maxPoints(i,2);

    z0 = J(x0,y0);

    gJ = gradJ(x0,y0);
    gh = gradh(x0,y0);

    % Normalizovane gradienty pre lepsiu vizualizaciu
    gJ_plot = gradientScale * gJ / norm(gJ);
    gh_plot = gradientScale * gh / norm(gh);

    % Gradient J
    quiver3( ...
        x0,y0,z0, ...
        gJ_plot(1),gJ_plot(2),0, ...
        0, ...
        '-r',...
        'LineWidth',2, ...
        'MaxHeadSize',0.8);

    % Gradient h
    quiver3( ...
        x0,y0,z0, ...
        gh_plot(1),gh_plot(2),0, ...
        0, ...
        ':g',...
        'LineWidth',2, ...
        'MaxHeadSize',0.8);

end

%% --- MINIMA ---

for i = 1:size(minPoints,1)

    x0 = minPoints(i,1);
    y0 = minPoints(i,2);

    z0 = J(x0,y0);

    gJ = gradJ(x0,y0);
    gh = gradh(x0,y0);

    % Normalizovane gradienty
    gJ_plot = gradientScale * gJ / norm(gJ);
    gh_plot = gradientScale * gh / norm(gh);

    % Gradient J
    quiver3( ...
        x0,y0,z0, ...
        gJ_plot(1),gJ_plot(2),0, ...
        0, ...
        '-r',...
        'LineWidth',2, ...
        'MaxHeadSize',0.8);

    % Gradient h
    quiver3( ...
        x0,y0,z0, ...
        gh_plot(1),gh_plot(2),0, ...
        0, ...
        ':g',...
        'LineWidth',2, ...
        'MaxHeadSize',0.8);

end

%% 9. Legenda gradientov

% Fiktivne objekty iba pre legendu
gJ_legend = quiver3( ...
    NaN,NaN,NaN, ...
    1,0,0, ...
    0, ...
    'LineWidth',3);

gh_legend = quiver3( ...
    NaN,NaN,NaN, ...
    1,0,0, ...
    0, ...
    'LineWidth',3);

%% 10. Nastavenie grafu

xlabel('x', ...
    'FontSize',12);

ylabel('y', ...
    'FontSize',12);

zlabel('J(x,y)', ...
    'FontSize',12);


legend( ...
    'J(x,y)', ...
    'Ohranicenie h(x,y)=0', ...
    'Maximum', ...
    'Minimum', ...
    '\nabla J', ...
    '\nabla h', ...
    'Location','best');

grid on;
%axis equal;

xlim([-2.8 2.8]);
ylim([-2.8 2.8]);

view(45,30);
