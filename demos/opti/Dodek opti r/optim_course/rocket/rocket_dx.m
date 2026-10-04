function dx = rocket_dx(x,u,theta)

% States
p   = x(1:3); 
v   = x(4:6);
phi = x(7);
psi = x(8);
q   = x(9);
r   = x(10);
m   = x(11);

% Inputs
T         = u(1);
delta_phi = u(2);
delta_psi = u(3);

% Parameters
g   = theta(1);
Ix  = theta(2);
Iy  = theta(3);
l   = theta(4);
Isp = theta(5);

% Rotation matrices
R_phi = rotX(phi);
R_psi = rotY(psi);

R_delta_phi = rotX(delta_phi);
R_delta_psi = rotY(delta_psi);

% Dynamics
dp = %TODO

dv = %TODO

dphi = %TODO
dpsi = %TODO

dq = %TODO

dr = %TODO

dm = %TODO

dx = [
    dp;
    dv;
    dphi;
    dpsi;
    dq;
    dr;
    dm
];


end