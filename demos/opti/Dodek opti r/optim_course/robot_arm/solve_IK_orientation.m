function [theta] = solve_IK_orientation(pr, Rr, theta, R, dR, T, tol,Lambda)


while true
  
    p0 = transform_pi_to_p0(theta, [0;0;0], length(R), R, T);
    rp = %TODO

    R_tot = get_R_tot(theta, R);
 
    Re=%TODO
    ro = %TODO
    Jp = get_Jacobian(theta, [0;0;0], R, dR, T);
   
    Jo = zeros(3, length(R));

    for i = 1:length(R)
        dR_tot = get_dR_tot(theta, R, dR, i);
       
        Jo(:, i) = %TODO
    end

    J = %TODO    
    delta_theta = %TODO
    theta = %TODO
    
    if (rp' * rp + ro' * ro) < tol
        break
    end
end

% Wrap joint angles to [0, 2*pi]
theta = mod(theta, 2*pi);


end