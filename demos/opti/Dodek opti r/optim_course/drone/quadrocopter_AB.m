function [A, B] = quadrocopter_AB(x, u, parameters)


m = parameters{1};
g = parameters{2};
J = parameters{3};

T   = u(1);
tau = u(2:4);

p     = x(1:3);
v     = x(4:6);

eta   = x(7:9);
omega = x(10:12);

phi   = eta(1);
theta = eta(2);
psi   = eta(3);


A = %TODO
B = %TODO

end