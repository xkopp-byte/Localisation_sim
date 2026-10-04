%% EKF_localization_main.m
close all; clear; clc;

addpath("..\misc");
addpath("..\geometry");

%% --- PARAMETERS ---
dt = 0.05;
sensor_range = 7.0; sensor_fov = deg2rad(120);

%% --- World & landmarks ---
world_limits = [0 30; 0 25];
rng(5);
clusters = [5 5; 25 5; 15 20];
landmarks = [];

for c = 1:size(clusters,1)
    landmarks = [landmarks, clusters(c,:)' + randn(2,5)*4.0];
end
num_landmarks = size(landmarks,2);

%% --- Robot states ---
x_true = [15;10;pi/4];
x_hat  = x_true + [1;-1;0.2];
P = diag([0.5,0.5,deg2rad(10)]).^2;

%% --- Noise ---
Q = diag([0.02^2,0.02^2,0.0003]);
R = diag([1.5,0.5]);
chol_Q = chol(Q,'lower'); chol_R = chol(R,'lower');

%% --- Control ---
v_r_cmd=0; omega_cmd=0; v_r=0; omega=0;
v_r_max=2; omega_max=1.2; accel=0.05; friction=0.95;
keys = struct('w',0,'s',0,'a',0,'d',0);

%% --- Setup figure & handles ---
[fig,h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text] = setup_figure_diff_drive(x_true,x_hat,landmarks,world_limits);

%% --- MAIN LOOP ---
step = 0;
while  1
    step = step + 1;

    %% --- Control updates ---
    [v_r_cmd,omega_cmd,v_r,omega] = control_update(keys,v_r,omega,v_r_max,omega_max,accel,friction);

    %% --- True motion ---
    w = chol_Q*randn(3,1);
    x_true = x_true + [v_r_cmd*dt*cos(x_true(3));
                       v_r_cmd*dt*sin(x_true(3));
                       omega_cmd*dt] + w;
    x_true(3) = wrapToPi(x_true(3));

    %% --- EKF prediction ---
    theta = x_hat(3);
    %A = %TODO
    
    %P = %TODO

    %x_hat = %TODO
    x_hat(3) = wrapToPi(x_hat(3));
    

    %% --- Detect visible landmarks ---
    visible = [];
    for i = 1:num_landmarks
        dx = landmarks(1,i)-x_true(1);
        dy = landmarks(2,i)-x_true(2);
        r = sqrt(dx^2+dy^2);
        if r > sensor_range, continue; end
        if abs(wrapToPi(atan2(dy,dx)-x_true(3))) > sensor_fov/2, continue; end
        visible = [visible i];
    end

    %% --- EKF update ---
    if ~isempty(visible)
        for k = 1:length(visible)
            idx = visible(k);
            x_i = landmarks(1,idx); 
            y_i = landmarks(2,idx);

            % Simulated noisy measurement
            dx_true = x_i - x_true(1);
            dy_true = y_i - x_true(2);
            r = sqrt(dx_true^2 + dy_true^2);
            b = wrapToPi(atan2(dy_true,dx_true) - x_true(3));
            y = [r;b] + chol_R*randn(2,1);
    
          
    
            % Predicted measurement
           % y_hat = %TODO
            
            y_hat(2) = wrapToPi(y_hat(2));
    
            % Correct Jacobian
           % C_i = %TODO
   
    
            % Kalman gain
           % K = %TODO
    
            % Update state
           % x_hat = %TODO
            x_hat(3) = wrapToPi(x_hat(3));
    
            % Update covariance
           % P = %TODO
        end
    end

    %% --- Visualization ---
    if mod(step,2)==0 && ishandle(fig)
        update_visuals_diff_drive(x_true,x_hat,P,visible,h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text,landmarks,step,v_r_cmd,omega_cmd);
        drawnow limitrate;
    end

    pause(dt*0.8);
end
