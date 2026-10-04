function [x_k, x_log, J_log, s_log] = optim_newton_raphson(J_fun, x_k, eps, log)

x_log=[];
J_log=[];
s_log=[];

while 1
    
    [J_k,g_xk,H] = J_fun(x_k);
    
    x_k = x_k - H\g_xk;
    
    if log
        x_log=[x_log;(x_k+H\g_xk)'];
        J_log=[J_log;J_k];
        s_log=[s_log;(-H\g_xk)'];
    end

    if g_xk'*g_xk < eps
        break;
    end
end

end