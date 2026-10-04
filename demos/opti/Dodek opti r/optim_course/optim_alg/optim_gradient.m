function [x_k, x_log, J_log, s_log] = optim_gradient(J_fun, x_0, eps, gamma, log)

x_log=[];
J_log=[];
s_log=[];
x_k=x_0;

while 1
    
    [J_k,g_xk,H] = J_fun(x_k);
    x_k= x_k - gamma * g_xk;
    
    if log
        x_log=[x_log;(x_k+gamma*g_xk)'];
        J_log=[J_log;J_k];
        s_log=[s_log;-g_xk'];
    end
   
    if g_xk'*g_xk < eps
        break;
    end
        
end
end
