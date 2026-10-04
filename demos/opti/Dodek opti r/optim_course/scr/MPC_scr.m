addpath("..\MPC") 

clear

con=true;

% System dynamics
A = [0.2, 0.8;
    -0.4, 0.5];
B = [2, 0.5;
     0, 1];

n = size(A,1);  % Number of states
r = size(B,2);  % Number of inputs

p = 10;  % Prediction horizon

% Get prediction matrices M and N
[M,N] = MPC_MN_state(A,B,p);

% Cost function weighting
Q = diag([1,1]);                % State weighting
Q_ = block_diag(Q, p);     % Block diagonal over horizon

R = diag([5,5]);           % Input weighting
R_ = block_diag(R, p);     % Block diagonal over horizon


x = [0;10];  % Initial state
if con 
 % H = %TODO
  % Input and state constraints
u_underbar = [-1; -1];
u_overbar  = [0; 0.8];
U_underbar = repmat(u_underbar, p, 1);
U_overbar  = repmat(u_overbar, p, 1);

x_underbar = [0; 0];
x_overbar  = [6; 5];
X_underbar = repmat(x_underbar, p, 1);
X_overbar  = repmat(x_overbar, p, 1);

% Build constraint matrices for QP
%A_con = %TODO
else    
%K = %TODO
end


n_steps = 15;  % Number of simulation steps

x_ = [];  % State history
u_ = [];  % Input history

for i = 1:n_steps

   % b = %TODO

    if con

    % Constraint bounds
   % b_con =%TODO

    % Solve QP
   % U = quadprog(%TODO);
   % u = U(%TODO); % First input from constrained control law
    else 
    %    u = %TODO
    end    

    
    x_ = [x_, x];
    u_ = [u_, u];

    x = A*x + B*u;  % State update
end
%%

style = '-k'; 

% Plot x1(k)
subplot(2,2,1)
stairs(0:(n_steps-1), x_(1,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('x_1(k)');
xlim([0,n_steps-1]);
grid on

% Plot x2(k)
subplot(2,2,2)
stairs(0:(n_steps-1), x_(2,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('x_2(k)');
xlim([0,n_steps-1]);
grid on

% Plot u1(k)
subplot(2,2,3)
stairs(0:(n_steps-1), u_(1,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('u_1(k)');
xlim([0,n_steps-1]);
grid on

% Plot u2(k)
subplot(2,2,4)
stairs(0:(n_steps-1), u_(2,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('u_2(k)');
xlim([0,n_steps-1]);
grid on

set(gcf,'position',[0,200,700,350]);