function dx = pendulum_propeler_model_dx(x,u,theta)

    a   = theta(1);
    alpha   = theta(2);
    beta    = theta(3);
    lambda1 = theta(4);
    lambda2 = theta(5);

    dx = zeros(3,1);

    dx(1) = %TODO
    dx(2) = %TODO
    dx(3) = %TODO

end