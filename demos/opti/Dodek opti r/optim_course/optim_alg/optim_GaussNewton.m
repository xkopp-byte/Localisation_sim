function [x_hat, g, H] = optim_GaussNewton(f, x_hat, tol,Lambda)

while 1
    [r, J] = f(x_hat);
    H = %TODO
   
    g = %TODO
   

    if g' * g < tol
        break;
    end

    delta = %TODO
    x_hat = %TODO

 
end

end