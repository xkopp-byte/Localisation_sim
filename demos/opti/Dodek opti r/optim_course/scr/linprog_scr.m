%%

addpath("..\fcn");
addpath("..\optim_alg");
addpath("..\misc");


%% Linprog problem

clc; clear; close all;


A = [2 1;
     1 2;
     1 1;
     -1, 0;
      0, -1];

b = [8; 8; 5; 0; 0];


c=[-3;-5];

xl=[0;-0];
xh=[5;5];

J = @(x)(fcn_LF(x,c));

x_0=[1;1];
lambda_0=[1;1;1;1;1];

[x_lp, x_log, s, lambda] = optim_linprog(x_0, lambda_0,c, A, b, 1e-6 , true)


plot_2D_lin_ineq(xl,xh,A,b)
hold on
plot_2D_optim(J,xl,xh,x_lp,[],x_log,[]);

quiver(x_lp(1),x_lp(2),[1,0]*c,[0,1]*c,0.3,'-r','LineWidth',1.5);
quiver(x_lp(1),x_lp(2),[1,0]*(A'*lambda),[0,1]*(A'*lambda),0.3,'-c','LineWidth',1.5);


