function [theta_hat, t_hat] = ICP_estimate_2D(pA, pB, theta_hat, t_hat, tol, tol_d, KDtree,Lambda)

i=0;
    while true
        % 2D rotation matrix from theta_hat (single angle)
        R_hat = %TODO
        

        % Transform points pA using current estimate
        pA_transformed = %TODO
        
        % Find nearest neighbors and distances using knnsearch
        [idx, d] = knnsearch(KDtree, pA_transformed,'k',1);

        % Estimate updated transform parameters with correspondences
        [theta_hat, t_hat, ~] = transform_estimate2D(%TODO);

        % Check convergence by mean distance
        i=i+1;
        if mean(d) < tol_d || i>200
            break;
        end
    end

end