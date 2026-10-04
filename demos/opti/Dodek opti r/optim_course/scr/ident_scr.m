
addpath("..\lab_models\");
addpath("..\ident\");
addpath("..\optim_alg\");
addpath("..\misc\");
%%

load("pendulum_ident_data.mat")
y=double(y);
y=deg2rad(y);
Ts=0.05;
theta_0=[20;   100;    1.5;    1.5; 0];
C=[0,1,0];


Lambda=diag([1;1;1e1;1e3;1e4]);



f_fun=@pendulum_propeler_model_dx;
A_fun=@pendulum_propeler_model_AB;
df_dtheta_fun=@pendulum_propeler_model_df_dtheta;


theta_hat = ident_sensitivity(y', theta_0, [0;0;0], Ts, u, C, f_fun,A_fun, df_dtheta_fun, 1e-2,Lambda)

x_hat = ode4input(f_fun, t, [0;0;0], u,theta_hat);

y_hat=C*x_hat;

figure;

subplot(2,1,1);
plot(t,y,'-k','LineWidth',1.5);
hold on
plot(t,y_hat,'--g','LineWidth',1.5);
xlabel('t[s]');
ylabel('\phi(t)')
legend('data','estimated model')
grid on

subplot(2,1,2);
plot(t,u,'-k','LineWidth',1.5);
xlabel('t[s]');
ylabel('u(t)')
grid on


