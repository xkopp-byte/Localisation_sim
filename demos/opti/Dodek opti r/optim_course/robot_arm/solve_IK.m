function [theta] = solve_IK(pr, theta, R, dR, T, tol,Lambda)



while true
    p0 = %TODO
    r = %TODO
    J = get_Jacobian(theta, [0;0;0], R, dR, T);
   
    delta_theta =%TODO;
    theta = %TODO;

    if (r' * r) < tol
        break
    end
end

% Wrap joint angles to [0, 2*pi]
theta = mod(theta, 2*pi);

end