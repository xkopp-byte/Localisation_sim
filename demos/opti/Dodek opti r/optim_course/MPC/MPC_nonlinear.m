function [U, lambda] = MPC_nonlinear(U, x_k, X_r, u0, R, Q, p , Ts, f_fun,AB_fun,theta, tol,Lambda, u_underbar,u_overbar, lambda_0)


    r = length(u0); 
    U_underbar = repmat(u_underbar, p, 1);
    U_overbar  = repmat(u_overbar, p, 1);

    A_con = %TODO
    b_con = %TODO

    J_fun=@(U)MPC_nonlinear_get_gH(U, x_k, X_r, u0, R, Q, p , Ts, f_fun,AB_fun,theta);
    [U,~,~,lambda]=optim_con(J_fun,U, lambda_0, A_con, b_con,tol, Lambda , 0);
  
end


function [J,g,H] = MPC_nonlinear_get_gH(U, x_k, X_r, u0, R, Q, p , Ts, f_fun,AB_fun,theta)


    J=[];
    n = length(x_k); 
    r = length(u0); 
    X=zeros(n*p,1);

      Jx=zeros(n*p,r*p);

      for i=1:p 
        S=zeros(n,r);
        for j=1:p  
            S=ode4step(%TODO);     
            Jx(1+(j-1)*n:j*n, 1+(i-1)*r:i*r) = S;
        end
      end

    H = %TODO
    g = %TODO
 
end

function dS = dS_fun_active(S,x,u ,theta,AB_fun)
    
    [A,B]=AB_fun(x, u, theta);
    dS = %TODO
end

function dS = dS_fun_inactive(S,x,u ,theta,AB_fun)
    
    [A,B]=AB_fun(x, u, theta);
    dS = %TODO
end
