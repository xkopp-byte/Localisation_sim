
addpath("..\lab_models\");
addpath("..\misc\");

%% LQ
close all
clear
n_steps=50;

Ts=0.05;

u0=30;
theta_hat=[15.8266   82.9652    1.0391    2.1091   -0.0117];
C=[0,1,0];

%x0=%TODO

%[Ac,Bc] = %TODO
%[A,B,C] = %TODO

y_ref=x0(2);

m=size(C,1);
r=size(B,2);
n=size(A,1);

%A_tilde=%TODO
%B_tilde=%TODO

%Q_=%TODO
%R_=%TODO

%Qz=%TODO
%Q_tilde=%TODO

%K_LQ=%TODO
%Kx=%TODO
%Kz=%TODO

P=zeros(n);

R=1e-4;
Q=diag([100;0.1;1])*1e-3;

z=zeros(m,1);

x_=[];
y_=[];
u_=[];

x=[0;0;0];

delta_x_hat=x-x0;
delta_u=-u0;

for i=1:n_steps
  
  
 %   delta_x_hat=%TODO

    y=C*x+sqrt(R)*randn();
 
  %  P=%TODO
  %  K=%TODO;
  %  delta_x_hat=%TODO

   % P=%TODO

   % delta_u=%TODO
    u=max(0,delta_u+u0);
    u=min(100,u);
    

    x=ode4step(@pendulum_propeler_model_dx,Ts,x,u*0.95,theta_hat)+chol(Q)*randn(3,1);
    
   % z=%TODO

    
    x_=[x_,x];
    u_=[u_,u];
end



figure
style='-k';

subplot(4,1,1)
stairs(0:(n_steps-1),x_(1,:),style,'LineWidth',1.5)
xlabel('k');
ylabel('x_1(k)');
xlim([0,n_steps-1]);
grid on

subplot(4,1,2)
stairs(0:(n_steps-1),x_(2,:),style,'LineWidth',1.5)
hold on
stairs(0:(n_steps-1),ones(1,n_steps)*y_ref,'--r','LineWidth',1.5)
xlabel('k');
ylabel('x_2(k)');
xlim([0,n_steps-1]);
grid on

subplot(4,1,3)
stairs(0:(n_steps-1),x_(3,:),style,'LineWidth',1.5)
xlabel('k');
ylabel('x_3(k)');
xlim([0,n_steps-1]);
grid on

subplot(4,1,4)
stairs(0:(n_steps-1),u_(:),style,'LineWidth',1.5)
xlabel('k');
ylabel('u(k)');
xlim([0,n_steps-1]);
grid on

grid on
set(gcf,'position',[0,200,650,400]);


