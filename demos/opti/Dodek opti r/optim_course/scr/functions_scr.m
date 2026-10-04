

addpath("..\fcn");

% Kvadratická funkcia

close all;
clear;

% A=[1,0.5;0.5,3];
% A=-[1.5,0.2;0.2,2];
A=[-2,0.5;0.5,3];

b=[-6;10];
c=10;

xl=[-15;-15];
xh=[15;15];

x_opt = -(A ^ -1) * b;
[Q,Lambda]= eig(A);

J_opt=fcn_QF(x_opt,A,b,c);

figure(1);

[x1,x2]=meshgrid(xl(1):0.3:xh(1),xl(2):0.3:xh(2));
J=zeros(size(x1));
dJ_dx1=zeros(size(x1));
dJ_dx2=zeros(size(x1));

hold on;
grid on;

for i=1:size(x1,1)
    for j=1:size(x1,2)
        x=[x1(i,j);x2(i,j)];
        
       [J(i,j),g]= fcn_QF(x,A,b,c);
    
        dJ_dx1(i,j)=g(1);
        dJ_dx2(i,j)=g(2);
    end
end

set(streamslice(x1,x2,dJ_dx1,dJ_dx2,1),'Color','r');

contour(x1,x2,J,50);
plot3(x_opt(1),x_opt(2),J_opt,'og','LineWidth',4,'MarkerSize',10);

axis equal;

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

xlabel('x_1');
ylabel('x_2');


figure(2);

mesh(x1,x2,J);
hold on

plot3(x_opt(1),x_opt(2),J_opt,'og','LineWidth',4,'MarkerSize',10);

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

xlabel('x_1');
ylabel('x_2');


%% Himmelblau funkcia

close all
clear;
clc;


xl = [-6; -6];
xh = [ 6;  6];

step = 0.1;


[x1,x2] = meshgrid( ...
    xl(1):step:xh(1), ...
    xl(2):step:xh(2));

J = zeros(size(x1));

dJ_dx1 = zeros(size(x1));
dJ_dx2 = zeros(size(x1));

for i = 1:size(x1,1)
    for j = 1:size(x1,2)

        x = [x1(i,j);
             x2(i,j)];

         [J(i,j),g]= fcn_himmelblau(x);

        dJ_dx1(i,j)=g(1);
        dJ_dx2(i,j)=g(2);
    end
end

figure(2);
clf;

hold on;
grid on;
box on;

set(streamslice( ...
    x1, x2, ...
    dJ_dx1, dJ_dx2, ...
    1), ...
    'Color', 'r');

contour(x1,x2,J,50,'LineWidth',1);

axis equal;

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

xlabel('x_1');
ylabel('x_2');

figure(3);

mesh(x1,x2,J);
hold on

plot3(x_opt(1),x_opt(2),J_opt,'og','LineWidth',4,'MarkerSize',10);

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

xlabel('x_1');
ylabel('x_2');

%%

% x0=[3.584458;-1.848126];
x0=[-3.0730   -0.0814];
% x0=[-0.2708   -0.9230];


[~,~,H0] = fcn_himmelblau(x0);



lambda=eig(H0)

plot(x0(1),x0(2),'om','LineWidth',4);

