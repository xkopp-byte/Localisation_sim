function [A,B] = rocket_AB(x,u,theta)

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

dR_phi = drotX(phi);
dR_psi = drotY(psi);

R_delta_phi = rotX(delta_phi);
R_delta_psi = rotY(delta_psi);

dR_delta_phi = drotX(delta_phi);
dR_delta_psi = drotY(delta_psi);

A = zeros(11,11);
%TODO

B = zeros(11,3);
%TODO

end