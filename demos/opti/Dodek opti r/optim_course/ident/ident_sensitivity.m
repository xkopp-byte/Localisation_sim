function theta_hat = ident_sensitivity(y, theta_0, x0, Ts, u, C, f_fun,A_fun, df_dtheta_fun, tol,Lambda)

    rJ_fun = @(theta_hat) get_rJ(theta_hat, y, x0, Ts, u, C, f_fun,A_fun, df_dtheta_fun);

    [theta_hat] = optim_GaussNewton(rJ_fun, theta_0, tol,Lambda);

end


function [r, J] = get_rJ(theta_hat, y, x0, Ts, u, C, f_fun,A_fun, df_dtheta_fun)

    n = length(x0);        
    p = length(theta_hat);
    N=length(y);

    odefun = @(xS, u,theta_hat) get_dxS(xS, u, theta_hat, f_fun,A_fun, df_dtheta_fun, n);

    xS = ode4input(%TODO);
    x = %TODO
    y_hat = %TODO

    r=%TODO
    S = %TODO

    J = zeros(N, p);

    for i = 1:N
        for j = 1:p
          J(i, j) = %TODO
        end
    end
  
end


function dxS = get_dxS(xS, u, theta, f_fun,A_fun, df_dtheta_fun, n)

    x = xS(1:n);
    S = xS(n + 1:end);
    p = length(theta);

    dx=%TODO 
    A=%TODO
    df_dtheta=%TODO

    dS = zeros(p * n, 1);
    for j = 1:p
          dS(1+n*(j-1):n*j) =%TODO;
    end

    dxS = [dx; dS];
end