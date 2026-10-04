function X = ode4input(f,t,X0,u,theta)
 
h = diff(t);

n = length(X0);
N = length(t);
X = zeros(n,N);

X(:,1) = X0;
for i = 2:N
  hi = h(i-1);
  xi = X(:,i-1);
  k1 = %TODO
  k2 = %TODO
  k3 = %TODO  
  k4 = %TODO
  X(:,i) = %TODO
end

