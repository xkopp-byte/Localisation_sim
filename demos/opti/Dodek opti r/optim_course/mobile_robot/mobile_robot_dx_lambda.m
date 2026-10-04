function dx_lambda = mobile_robot_dx_lambda(t, x_lambda, q, rv, romega)
    
    % Unpack state variables
    x_pos = x_lambda(1);
    y_pos = x_lambda(2);
    theta = x_lambda(3);
    
    % Unpack costate variables
    lambda_x = x_lambda(4);
    lambda_y = x_lambda(5);
    lambda_theta = x_lambda(6);
    
    % Compute optimal controls based on current state and costates
    [v, omega] = mobile_robot_get_optimal_u(theta, rv, romega, lambda_x, lambda_y, lambda_theta);
    
    % System dynamics (robot kinematics)
    dx = %TODO
    dy =  %TODO
    dtheta = %TODO
    
    % Costate dynamics (derived from Hamiltonian system)
    dlambda_x = %TODO
    dlambda_y = %TODO
    dlambda_theta = %TODO
    
    % Pack derivatives into output vector
    dx_lambda = [dx; dy; dtheta; dlambda_x; dlambda_y; dlambda_theta];
end

