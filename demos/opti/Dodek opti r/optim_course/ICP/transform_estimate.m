function [theta_hat,t_hat] = transform_estimate(pA,pB,theta_hat,t_hat,tol,Lambda)

x_hat = optim_GaussNewton(@(x_hat)get_rJ(x_hat,pA,pB),[theta_hat;t_hat],tol,Lambda);

theta_hat=%TODO;
t_hat=%TODO;

end


function [r,J] = get_rJ(x_hat,pA,pB)


theta_hat=%TODO
t_hat=%TODO

N=size(pA,1);
R_hat=%TODO

d_R_hat_1=%TODO
d_R_hat_2=%TODO
d_R_hat_3=%TODO

r = %TODO
df_d_theta_1 = %TODO
df_d_theta_2 = %TODO
df_d_theta_3 = %TODO

J=%TODO

end