function [theta_hat,t_hat,H] = transform_estimate2D(pA,pB,theta_hat,t_hat,tol,Lambda)

[x_hat,~,H] = optim_GaussNewton(@(x_hat)get_rJ(x_hat,pA,pB),[theta_hat;t_hat],tol,Lambda);

theta_hat=%TODO
t_hat=%TODO

end


function [r,J] = get_rJ(x_hat,pA,pB)


theta_hat=%TODO
t_hat=%TODO

N=size(pA,1);

R_hat=%TODO
d_R_hat=%TODO

r = %TODO
df_d_theta=%TODO
J=%TODO

end