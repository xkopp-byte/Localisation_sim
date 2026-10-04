function [x, x_log, s, lambda] = optim_quadprog(x_0, lambda_0, A, b, A_c, b_c, eps, log)

x_log = [];

n_params = size(A, 2);
n_con = size(A_c, 1);

theta = zeros(n_params + n_con + n_con, 1);
theta(1:n_params) = x_0;
theta(n_params+1:n_params+n_con) = lambda_0;
theta(n_params+n_con+1:end) = -(A_c * x_0 - b_c);

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

    if sum(theta(n_params+1:end) + delta(n_params+1:end) < 0)
        i = (1:length(theta)) > n_params & theta' + delta' < 0 & delta' < 0;
        gamma = 0.9 * min(-theta(i) ./ delta(i));
    end    
    
    if %TODO
        break;
    end

    theta = %TODO
end

end