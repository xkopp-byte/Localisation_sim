addpath("..\fcn");
addpath("..\optim_alg");
addpath("..\misc");

%% Himmelblau function
clear;
J = @(x)(fcn_himmelblau(x));
xl=[-5;-5];
xh=[5;5];
x_anal=[3,2;-2.805118,3.131312;-3.779310,-3.283186;3.584458,-1.848126]';
plot_2D_function(J,xl,xh);
%% Nelder-Mead method
s0=[xl(1),xl(2);xl(1),xh(2);xh(1),(xl(2)+xh(2))/2];

alpha=0.75;
beta=1.5;
gamma=0.5;
delta=0.25;

 % alpha=0.5;
 % beta=1.2;
 % gamma=0.7;
 % delta=0.7;


eps=1e-4;
[x_nelder_mead,x_log,J_log] = optim_nelder_mead(J,s0,alpha,beta,gamma,delta,eps,true);
close all
for i=1:3:size(x_log,1)-6
        plot_2D_optim(J,xl,xh,x_nelder_mead,x_anal,[],[]);
        i=i+3;
        plot([x_log(i,1),x_log(i+1,1)],[x_log(i,2),x_log(i+1,2)],'-og','LineWidth',1.5);
        plot([x_log(i,1),x_log(i+2,1)],[x_log(i,2),x_log(i+2,2)],'-og','LineWidth',1.5);
        plot([x_log(i+1,1),x_log(i+2,1)],[x_log(i+1,2),x_log(i+2,2)],'-og','LineWidth',1.5);
        i=i-3;

        plot([x_log(i,1),x_log(i+1,1)],[x_log(i,2),x_log(i+1,2)],'-ob','LineWidth',1.5);
        plot([x_log(i,1),x_log(i+2,1)],[x_log(i,2),x_log(i+2,2)],'-ob','LineWidth',1.5);
        plot([x_log(i+1,1),x_log(i+2,1)],[x_log(i+1,2),x_log(i+2,2)],'-ob','LineWidth',1.5);

        legend off
        pause(0.5)
      
end




