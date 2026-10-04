function X_lambda = mobile_robot_initial_guess(t,x0,y0,theta0,T)

    % state guess
    x = x0*(1-t/T);
    y = y0*(1-t/T);
    theta = theta0*(1-t/T);

    % costate guess
    lambda_x = zeros(size(t));
    lambda_y = zeros(size(t));
    lambda_theta = zeros(size(t));

    X_lambda = [
        x;
        y;
        theta;
        lambda_x;
        lambda_y;
        lambda_theta
    ];

end