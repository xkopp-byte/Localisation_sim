clc; clear; close all;
addpath("..\drone");
addpath("..\MPC");
addpath("..\misc");
addpath("..\geometry");

%% ================================================================
% QUADROTOR PARAMETERS
% ================================================================

h = 0.02;                  % simulation step [s]
Ts = 0.2;                  % control step [s]
Tf = 50.0;                 % final time [s]

t = 0:h:Tf;
N = length(t);

m = 1.20;              % [kg]
g = 9.81;              % [m/s^2]

l = 0.23;              % [m]

J = diag([ ...
    0.014, ...
    0.014, ...
    0.025]);

kT = 1.5e-5;           % [N/(rad/s)^2]
kQ = 2.5e-7;           % [Nm/(rad/s)^2]

parameters = {m, g, J, l, kT, kQ};


%u0=%TODO


%R=%TODO
%Q_p=%TODO
%Q_v=%TODO
%Q_eta=%TODO
%Q_omega=%TODO
Q=diag([Q_p;Q_v;Q_eta;Q_omega]);

x0=zeros(12,1);

%[Ac,Bc] = %TODO
%[A,B,~] = %TODO

p = 20;

[M_MPC,N_MPC] = MPC_MN_state(A,B,p);

%Q_ = %TODO
%R_ = %TODO

%H = %TODO

u_underbar = [-3;-0.1;-0.1;-0.1];
u_overbar  = [3;0.1;0.1;0.1];

U_underbar = repmat(u_underbar, p, 1);
U_overbar  = repmat(u_overbar, p, 1);

%A_con =%TODO
%b_con = %TODO


V = [ ...
    1.0  0.75  0.5;
    3.0  0.75  0.5;
    3.0  2.25  0.5;
    1.0  2.25  0.5;
    1.0  0.75  2.5;
    3.0  0.75  2.5;
    3.0  2.25  2.5;
    1.0  2.25  2.5;
    1.0  0.75  0.5;
    3.0  0.75  2.5;
    3.0  2.25  0.5;
    1.0  0.75  2.5];



%% ================================================================
% MEMORY ALLOCATION
% ================================================================

X = zeros(12,N);

X(:,1) =  [ ...
    0;          % x
    0;          % y
    0;        % z
    ...
    0;          % vx
    0;          % vy
    0;          % vz
    ...
    0;          % phi
    0;          % theta
    0;          % psi
    ...
    0;          % wx
    0;          % wy
    0];         % wz;

U = zeros(4,N);



%% ================================================================
% SIMULATION LOOP
% ================================================================

for k = 1:N-1

    % ------------------------------------------------------------
    % Current simulation time
    % ------------------------------------------------------------

    tk = t(k);

    if(mod(tk,Ts)==0)

    iV=ceil(k/N*size(V,1));
    xr=x0;
    xr(1:3,:)=V(iV,:)';

   % b = %TODO


   % u_ = quadprog(%TODO);
   % u = %TODO

    end

    U(:,k) = u;

    X(:,k+1) = ode4step( ...
        @quadrocopter_dx, ...
        h, ...
        X(:,k), ...
        u, ...
        parameters);

end


% Store last input for plotting
U(:,N) = U(:,N-1);


%% ================================================================
% EXTRACT STATES
% ================================================================

p = X(1:3,:).';

v = X(4:6,:).';

eta = X(7:9,:).';

omega = X(10:12,:).';


%% ================================================================
% EULER ANGLES IN DEGREES
% ================================================================

eta_deg = rad2deg(eta);


%% ================================================================
% POSITION
% ================================================================

figure('Name','Quadrocopter Position');

subplot(3,1,1)

plot(t,p(:,1),'LineWidth',1.5);

grid on;

ylabel('x [m]');

title('Position');


subplot(3,1,2)

plot(t,p(:,2),'LineWidth',1.5);

grid on;

ylabel('y [m]');


subplot(3,1,3)

plot(t,p(:,3),'LineWidth',1.5);

grid on;

ylabel('z [m]');

xlabel('Time [s]');


%% ================================================================
% VELOCITY
% ================================================================

figure('Name','Quadrocopter Velocity');

subplot(3,1,1)

plot(t,v(:,1),'LineWidth',1.5);

grid on;

ylabel('v_x [m/s]');

title('Velocity');


subplot(3,1,2)

plot(t,v(:,2),'LineWidth',1.5);

grid on;

ylabel('v_y [m/s]');


subplot(3,1,3)

plot(t,v(:,3),'LineWidth',1.5);

grid on;

ylabel('v_z [m/s]');

xlabel('Time [s]');


%% ================================================================
% ATTITUDE
% ================================================================

figure('Name','Quadrocopter Attitude');

subplot(3,1,1)

plot(t,eta_deg(:,1),'LineWidth',1.5);

grid on;

ylabel('\phi [deg]');

title('Euler angles');


subplot(3,1,2)

plot(t,eta_deg(:,2),'LineWidth',1.5);

grid on;

ylabel('\theta [deg]');


subplot(3,1,3)

plot(t,eta_deg(:,3),'LineWidth',1.5);

grid on;

ylabel('\psi [deg]');

xlabel('Time [s]');


%% ================================================================
% ANGULAR VELOCITY
% ================================================================

figure('Name','Angular Velocity');

subplot(3,1,1)

plot(t,omega(:,1),'LineWidth',1.5);

grid on;

ylabel('\omega_x [rad/s]');

title('Body angular velocity');


subplot(3,1,2)

plot(t,omega(:,2),'LineWidth',1.5);

grid on;

ylabel('\omega_y [rad/s]');


subplot(3,1,3)

plot(t,omega(:,3),'LineWidth',1.5);

grid on;

ylabel('\omega_z [rad/s]');

xlabel('Time [s]');


%% ================================================================
% CONTROL INPUTS
% ================================================================

figure('Name','Control Inputs');

subplot(4,1,1)

plot(t,U(1,:),'LineWidth',1.5);

grid on;

ylabel('T [N]');

title('Control inputs');


subplot(4,1,2)

plot(t,U(2,:),'LineWidth',1.5);

grid on;

ylabel('\tau_x [Nm]');


subplot(4,1,3)

plot(t,U(3,:),'LineWidth',1.5);

grid on;

ylabel('\tau_y [Nm]');


subplot(4,1,4)

plot(t,U(4,:),'LineWidth',1.5);

grid on;

ylabel('\tau_z [Nm]');

xlabel('Time [s]');

%% ---------------------------------------------------------------
% ANIMATION SETTINGS
% ---------------------------------------------------------------

animationFPS = 30;
frameStep = max(1,round(1/(animationFPS*Ts)));

% Set to true for approximately real-time playback
realTimePlayback = false;

% Visual scale of coordinate systems
bodyAxisLength  = 0.6;
worldAxisLength = 0.6;



%% ---------------------------------------------------------------
% ROTOR LOCATIONS
%
% These locations correspond to your allocation matrix:
%
% tau_x = [ 0   l   0  -l ] * F
% tau_y = [-l   0   l   0 ] * F
%
% Therefore:
%
% motor 1 : +x
% motor 2 : +y
% motor 3 : -x
% motor 4 : -y
% ---------------------------------------------------------------

motorPositionBody = [ ...
    l,  0,  0;
    0,  l,  0;
    -l,  0,  0;
    0, -l,  0];


%% ================================================================
% FIGURE 1 -- QUADROTOR
% ================================================================

figDrone = figure( ...
    'Name','Quadrotor 3D Animation', ...
    'Color',[0 0 0], ...
    'NumberTitle','off');

axDrone = axes(figDrone);
hold(axDrone,'on');
grid(axDrone,'on');
axis(axDrone,'equal');
view(axDrone,35,25);

xlabel(axDrone,'X_W [m]');
ylabel(axDrone,'Y_W [m]');
zlabel(axDrone,'Z_W [m]');

set(axDrone, ...
    'Color',[0.08 0.09 0.11], ...
    'XColor','w', ...
    'YColor','w', ...
    'ZColor','w', ...
    'GridColor',[0.35 0.35 0.35], ...
    'FontSize',10);


%% ---------------------------------------------------------------
% FIXED WORLD COORDINATE SYSTEM
% ---------------------------------------------------------------

quiver3(axDrone, ...
    0,0,0, ...
    worldAxisLength,0,0, ...
    0, ...
    'Color',[0.9 0.2 0.2], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.35);

quiver3(axDrone, ...
    0,0,0, ...
    0,worldAxisLength,0, ...
    0, ...
    'Color',[0.2 0.9 0.2], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.35);

quiver3(axDrone, ...
    0,0,0, ...
    0,0,worldAxisLength, ...
    0, ...
    'Color',[0.2 0.5 1.0], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.35);

text(axDrone, ...
    worldAxisLength,0,0, ...
    'X_W', ...
    'Color','w');

text(axDrone, ...
    0,worldAxisLength,0, ...
    'Y_W', ...
    'Color','w');

text(axDrone, ...
    0,0,worldAxisLength, ...
    'Z_W', ...
    'Color','w');


%% ---------------------------------------------------------------
% TRAJECTORY
% ---------------------------------------------------------------

trajectory = plot3(axDrone, ...
    p(1,1),p(1,2),p(1,3), ...
    'Color',[0.95 0.75 0.1], ...
    'LineWidth',2);


%% ---------------------------------------------------------------
% CURRENT POSITION MARKER
% ---------------------------------------------------------------

positionMarker = plot3(axDrone, ...
    p(1,1),p(1,2),p(1,3), ...
    'o', ...
    'MarkerSize',7, ...
    'MarkerFaceColor',[1 0.8 0.1], ...
    'MarkerEdgeColor','w');


%% ---------------------------------------------------------------
% QUADROTOR GRAPHICS GROUP
%
% hgtransform allows the entire drone model to be moved and rotated
% using a single homogeneous transformation matrix.
% ---------------------------------------------------------------

droneTransform = hgtransform('Parent',axDrone);

%% ---------------------------------------------------------------
% TOP BODY PLATE
% ---------------------------------------------------------------

plateLength = 0.30;
plateWidth  = 0.22;
plateHeight = 0.025;

[Xp,Yp] = meshgrid( ...
    [-plateLength/2 plateLength/2], ...
    [-plateWidth/2 plateWidth/2]);

Zp = (plateHeight/2)*ones(size(Xp));

surf(axDrone, ...
    Xp,Yp,Zp, ...
    'Parent',droneTransform, ...
    'FaceColor',[0 0.8 0.8], ...
    'EdgeColor','none');


%% ================================================================
% ARMS
% ================================================================

armColor = [0.65 0.67 0.72];

for i = 1:4

    P = motorPositionBody(i,:);

    line(axDrone, ...
        [0 P(1)], ...
        [0 P(2)], ...
        [0 P(3)], ...
        'Parent',droneTransform, ...
        'Color',armColor, ...
        'LineWidth',8);

end


%% ================================================================
% MOTOR PODS
% ================================================================

motorRadius = 0.035;
motorHeight = 0.045;

motorHandles = gobjects(4,1);

for i = 1:4

    [Xm,Ym,Zm] = cylinder(motorRadius,24);

    Zm = Zm*motorHeight - motorHeight/2;

    Xm = Xm + motorPositionBody(i,1);
    Ym = Ym + motorPositionBody(i,2);

    motorHandles(i) = surf(axDrone, ...
        Xm,Ym,Zm, ...
        'Parent',droneTransform, ...
        'FaceColor',[0.5 0.04 0.6], ...
        'EdgeColor','none', ...
        'FaceLighting','gouraud');

end


%% ================================================================
% PROPELLERS
%
% Two crossed blades per rotor.
% ================================================================

propellerLength = 0.105;

propellerHandles = gobjects(8,1);

for i = 1:4

    P = motorPositionBody(i,:);

    h1 = line(axDrone, ...
        P(1)+[-propellerLength propellerLength], ...
        [P(2) P(2)], ...
        [0.045 0.045], ...
        'Parent',droneTransform, ...
        'Color',[0.9 0.9 0.92], ...
        'LineWidth',3);

    h2 = line(axDrone, ...
        [P(1) P(1)], ...
        P(2)+[-propellerLength propellerLength], ...
        [0.045 0.045], ...
        'Parent',droneTransform, ...
        'Color',[0.9 0.9 0.92], ...
        'LineWidth',3);

    propellerHandles(2*i-1) = h1;
    propellerHandles(2*i)   = h2;

end


%% ================================================================
% BODY COORDINATE SYSTEM
%
% These are drawn in WORLD coordinates and updated every frame.
% This makes the motion of the body frame extremely clear.
% ================================================================

hBodyX = quiver3(axDrone, ...
    0,0,0, ...
    bodyAxisLength,0,0.2, ...
    0, ...
    'Color',[1 0.15 0.15], ...
    'LineWidth',2.0, ...
    'MaxHeadSize',0.45);

hBodyY = quiver3(axDrone, ...
    0,0,0, ...
    0,bodyAxisLength,0.2, ...
    0, ...
    'Color',[0.2 1 0.2], ...
    'LineWidth',2.0, ...
    'MaxHeadSize',0.45);

hBodyZ = quiver3(axDrone, ...
    0,0,0, ...
    0,0,bodyAxisLength+0.2, ...
    0, ...
    'Color',[0.25 0.55 1], ...
    'LineWidth',2.0, ...
    'MaxHeadSize',0.45);

plot3(V(:,1),V(:,2),V(:,3),'or');


bodyXLabel = text(axDrone,0,0,0,'x_B', ...
    'Color',[1 0.2 0.2]);

bodyYLabel = text(axDrone,0,0,0,'y_B', ...
    'Color',[0.2 1 0.2]);

bodyZLabel = text(axDrone,0,0,0,'z_B', ...
    'Color',[0.3 0.6 1]);


%% ================================================================
% GROUND PLANE
% ================================================================

allX = [p(:,1); 0];
allY = [p(:,2); 0];
allZ = [p(:,3); 0];

margin = 0.6 + l;

xmin = min(allX)-margin;
xmax = max(allX)+margin;

ymin = min(allY)-margin;
ymax = max(allY)+margin;

zmin = min(allZ)-margin;
zmax = max(allZ)+margin;

[Xg,Yg] = meshgrid( ...
    linspace(xmin,xmax,2), ...
    linspace(ymin,ymax,2));

Zg = zeros(size(Xg));

surf(axDrone, ...
    Xg,Yg,Zg, ...
    'FaceColor',[0.13 0.14 0.16], ...
    'FaceAlpha',0.45, ...
    'EdgeColor',[0.25 0.25 0.27]);


%% ---------------------------------------------------------------
% FIXED AXES LIMITS
%
% Important: the axes do NOT move during the animation.
% ---------------------------------------------------------------

xlim(axDrone,[xmin xmax]);
ylim(axDrone,[ymin ymax]);
zlim(axDrone,[zmin zmax]);


%% ---------------------------------------------------------------
% LIGHTING
% ---------------------------------------------------------------
%
% camlight(axDrone,'headlight');
% camlight(axDrone,'right');
% lighting(axDrone,'gouraud');


%% ================================================================
% ANIMATION LOOP
% ================================================================

for k = 1:frameStep:N

    %% ------------------------------------------------------------
    % CURRENT STATE
    % ------------------------------------------------------------

    position = X(1:3,k);

    phi   = X(7,k);
    theta = X(8,k);
    psi   = X(9,k);


    %% ------------------------------------------------------------
    % ROTATION MATRIX
    %
    % ZYX Euler angle convention:
    %
    % R = Rz(psi) Ry(theta) Rx(phi)
    %
    % Body vectors are transformed into WCS using:
    %
    % v_W = R_WB * v_B
    % -----------------------------------------------------------

    % Body -> world
    R = rotZ(psi)*rotY(theta)*rotX(phi);

    %% ------------------------------------------------------------
    % TRANSFORM COMPLETE DRONE
    % ------------------------------------------------------------

    H = [ ...
        R,          position;
        0 0 0,      1];

    set(droneTransform,'Matrix',H);


    %% ------------------------------------------------------------
    % BODY FRAME IN WCS
    % ------------------------------------------------------------

    ex = R(:,1);
    ey = R(:,2);
    ez = R(:,3);

    set(hBodyX, ...
        'XData',position(1), ...
        'YData',position(2), ...
        'ZData',position(3), ...
        'UData',bodyAxisLength*ex(1), ...
        'VData',bodyAxisLength*ex(2), ...
        'WData',bodyAxisLength*ex(3));

    set(hBodyY, ...
        'XData',position(1), ...
        'YData',position(2), ...
        'ZData',position(3), ...
        'UData',bodyAxisLength*ey(1), ...
        'VData',bodyAxisLength*ey(2), ...
        'WData',bodyAxisLength*ey(3));

    set(hBodyZ, ...
        'XData',position(1), ...
        'YData',position(2), ...
        'ZData',position(3), ...
        'UData',bodyAxisLength*ez(1), ...
        'VData',bodyAxisLength*ez(2), ...
        'WData',bodyAxisLength*ez(3));


    %% ------------------------------------------------------------
    % BODY AXIS LABELS
    % ------------------------------------------------------------

    px = position + 1.15*bodyAxisLength*ex;
    py = position + 1.15*bodyAxisLength*ey;
    pz = position + 1.15*bodyAxisLength*ez;

    set(bodyXLabel, ...
        'Position',px);

    set(bodyYLabel, ...
        'Position',py);

    set(bodyZLabel, ...
        'Position',pz);

    %% ------------------------------------------------------------
    % TRAJECTORY
    % ------------------------------------------------------------

    set(trajectory, ...
        'XData',p(1:k,1), ...
        'YData',p(1:k,2), ...
        'ZData',p(1:k,3));

    set(positionMarker, ...
        'XData',position(1), ...
        'YData',position(2), ...
        'ZData',position(3));

    title(axDrone, ...
        sprintf('Quadrotor  |  t = %.2f s', ...
        t(k)), ...
        'Color','w');

    drawnow;


    %% ------------------------------------------------------------
    % OPTIONAL REAL-TIME PLAYBACK
    % ------------------------------------------------------------

    if realTimePlayback && k < N

        pause(frameStep*Ts);

    end

end

