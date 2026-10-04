function [x,x_log,J_log] = optim_nelder_mead(J,s0,alpha,beta,gamma,delta,eps,log)

x_log=[];
J_log=[];
ns=size(s0,1); % Number of simplex points = number of variables + 1
s=s0;

Js=update_Js(s,J);  % Evaluate objective function at each simplex point
[Js, order] = sort(Js);
s = s(order,:);     % Sort simplex points by objective value (ascending)
while true
   

    if var(Js)<eps      % Check convergence (variance of function values)
        x=s(1,:)';      % Return the best vertex as the solution
        break;
    end

    if log
        x_log=[x_log;s];
        J_log=[J_log;Js(1)];
    end

    xt=%TODO          % Compute centroid of best ns-1 points
    x=%TODO       % Reflect worst point through the centroid

    Jx=J(x');                      % Evaluate objective at reflected point
    p=find(Jx<Js,1);               % Check for improvement

    if ~isempty(p)
        xh=%TODO  % Expansion step
        disp("Expand")
    else
        xh=%TODO  % Contraction step
        disp("Contract")
    end

    Jxh=J(xh');
    if Jxh<Jx
        x=xh;
        p=find(Jxh<Js,1);
        Jx=Jxh;
    end

    if ~isempty(p)
        s(p:ns,:)=%TODO % Insert improved point into simplex
        Js(p:ns,:)=%TODO % Insert improved point into simplex
    else
        s(2:ns,:)=%TODO % Reduce simplex
        Js(2:ns)=update_Js(s(2:ns,:),J);  % Evaluate objective function at each simplex point
        [Js, order] = sort(Js);
        s = s(order,:);     % Sort simplex points by objective value (ascending)
        disp("Reduce")
    end
end

end


function Js=update_Js(s,J)
% UPDATE_JS Evaluate the objective function J at each row of simplex s.
Js=zeros(size(s,1),1);
for q=1:size(s,1)
    Js(q)=J(s(q,:)');
end
end