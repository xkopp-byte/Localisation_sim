function [x, x_log, s, lambda] = optim_linprog(x_0, lambda_0,c, A, b, eps, log)

x_log = [];

n_params = length(c);
n_con = size(A, 1);

theta = zeros(n_params + n_con + n_con, 1);
theta(1:n_params) = %TODO
theta(n_params+1:n_params+n_con) = %TODO
theta(n_params+n_con+1:end) =  %TODO


while 1
    x = theta(1:n_params);
    lambda = theta(n_params+1:n_params+n_con);
    s = theta(n_params+n_con+1:end);

    if log
        x_log = [x_log; x'];
    end   
    
    mu=lambda'*s/n_con/2;

    r_1 = %TODO
    r_2 = %TODO
    r_3 = %TODO

    r = [r_1; r_2; r_3];
    
    J=%TODO
    delta = %TODO;

    gamma = 1;


    if sum(theta + delta< 0)
        %TODO
    end    
    
     if %TODO
        break;
    end

   
    theta = %TODO
end

end