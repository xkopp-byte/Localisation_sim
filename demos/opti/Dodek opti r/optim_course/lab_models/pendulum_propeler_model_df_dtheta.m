
function df_dtheta = pendulum_propeler_model_df_dtheta(x,u,theta)

    a   = theta(1);
    alpha   = theta(2);
    beta    = theta(3);
    lambda1 = theta(4);
    lambda2 = theta(5);

    df_da = %TODO
    df_dalpha = %TODO
    df_dbeta = %TODO
    df_dlambda1 = %TODO
    df_dlambda2 = %TODO
    
    df_dtheta=[df_da,df_dalpha,df_dbeta,df_dlambda1,df_dlambda2];
 
end
