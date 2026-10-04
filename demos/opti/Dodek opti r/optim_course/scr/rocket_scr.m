

clear

addpath("..\rocket\");
addpath("..\geometry\");
addpath("..\optim_alg\");
addpath("..\MPC\");
addpath("..\misc\");

%%

Ts = 0.15;
h=Ts/15;
Tf = 7;
N = round(Tf/h);

theta = [
    9.81;       % g [m/s^2]
    3.0;      % Ix [kg m^2]
    3.0;      % Iy [kg m^2]
    0.4;       % distance COM-engine [m]
    100         % Isp [s]
    ];

m0 = 10;      % kg
x = zeros(11,1);
x(11) = m0;
xr=x;
xr(3)=1;

X = zeros(11,N+1);


x(1:3)=[40;25;110];
x(4:6)=[-5;-10;-20];
x(7)=0.7;
x(8)=-0.6;
X(:,1)=x;
x0=x;

p=35;
tol=1e-2;


n=11;
r=3;

u_underbar = [0; -deg2rad(5); -deg2rad(5)];
u_overbar  = [180; deg2rad(5); deg2rad(5)];

Lambda=diag(repmat([1e-1;1;1],p,1))*0.01;

U = zeros(r,N);
U_MPC=zeros(r*p,1);

Xr=zeros(n,(N+p));

q=1;
for k=1:(N+p)
    Xr(:,k)=q*x0+((1-q)*xr);
    q=q*0.99;
end

lambda0=[10*ones(r*p,1);10*ones(r*p,1)];


for k=1:N

    t=(k-1)*h;

    if mod((k-1)*h,Ts)==0
        T_hover = x(11)*theta(1);
        u0=[T_hover;0;0];
    
        Q=diag([0.035*(x0(3)/(x(3)+5))^3*[1;1;1];ones(3,1)*1e-1;ones(2,1)*1e0;ones(2,1)*1e2;1e1]);
        R=diag([1e-3;1e3;1e3]);
        
        Q_=block_diag(Q,p);
        R_=block_diag(R,p);
      
        [U_MPC,lambda0]=MPC_nonlinear([U_MPC(1+r:p*r);U_MPC(1+(p-1)*r:p*r)], x, reshape(Xr(:,k:(k+p-1)),[n*p,1]), u0, R_, Q_, p ,Ts,@(x,u,theta)rocket_dx(x,u,theta),@(x,u,theta)rocket_AB(x,u,theta), theta, tol,Lambda,u_underbar,u_overbar,lambda0);
        
    end

    U(:,k)=U_MPC(1:r);

    x=ode4step(@rocket_dx,h,x,U(:,k)',theta);

    X(:,k+1)=x;

end

%%

close all
figure;
axis equal
grid on
view(45,30)

xlabel('X')
ylabel('Y')
zlabel('Z')

hold on

rocketLength = 15;

for k=1:3:N+1

    clf

    hold on
    grid on
    axis equal


    x=X(1,k);
    y=X(2,k);
    z=X(3,k);

    phi=X(7,k);
    psi=X(8,k);

    R=rotY(psi)*rotX(phi);
    body=R*[0;0;1];

    T=U(1,max(k-1,1));
    delta_phi=U(2,max(k-1,1));
    delta_psi=U(3,max(k-1,1));

    thrust=T*R*rotY(delta_psi)*rotX(delta_phi)*[0;0;-1];


    plot3(X(1,1:k),X(2,1:k),X(3,1:k),'b','LineWidth',1)

    quiver3( ...
        x,y,z,...
        rocketLength*body(1),...
        rocketLength*body(2),...
        rocketLength*body(3),...
        0,...
        'k','LineWidth',3);

    % thrust vector

    quiver3(...
        x,y,z,...
        0.1*thrust(1),...
        0.1*thrust(2),...
        0.1*thrust(3),...
        1,...
        'r','LineWidth',2);

    plot3(x,y,z,'og','LineWidth',3)
    plot3(xr(1),xr(2),xr(3),'ob','LineWidth',3)

    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    axis equal

    xlim([-50 50])
    ylim([-50 50])
    zlim([-5 120])
    view(45,30)

    drawnow

end


%%
close all
% Time vector
time = 0:h:Tf;

% State plots

figure;

subplot(4,3,1)
plot(time,X(1,:))
grid on
ylabel('x [m]')


subplot(4,3,2)
plot(time,X(2,:))
grid on
ylabel('y [m]')

subplot(4,3,3)
plot(time,X(3,:))
grid on
ylabel('z [m]')


subplot(4,3,4)
plot(time,X(4,:))
grid on
ylabel('v_x [m/s]')

subplot(4,3,5)
plot(time,X(5,:))
grid on
ylabel('v_y [m/s]')

subplot(4,3,6)
plot(time,X(6,:))
grid on
ylabel('v_z [m/s]')


subplot(4,3,7)
plot(time,rad2deg(X(7,:)))
grid on
ylabel('\phi [deg]')

subplot(4,3,8)
plot(time,rad2deg(X(8,:)))
grid on
ylabel('\psi [deg]')

subplot(4,3,10)
plot(time,rad2deg(X(9,:)))
grid on
ylabel('q [deg/s]')


subplot(4,3,11)
plot(time,rad2deg(X(10,:)))
grid on
ylabel('r [deg/s]')

subplot(4,3,12)
plot(time,X(11,:))
grid on
ylabel('m [kg]')

xlabel('Time [s]')



% Input plots

figure;

subplot(3,1,1)
stairs(time(1:end-1),U(1,:))
grid on
ylabel('T [N]')


subplot(3,1,2)
plot(time(1:end-1),rad2deg(U(2,:)))
grid on
ylabel('\delta_\phi [deg]')

subplot(3,1,3)
plot(time(1:end-1),rad2deg(U(3,:)))
grid on
ylabel('\delta_\psi [deg]')
xlabel('Time [s]')