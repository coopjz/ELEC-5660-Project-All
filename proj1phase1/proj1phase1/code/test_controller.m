%% test_controller_plots.m
% This script tests the outer-loop (position) and inner-loop (attitude)
% parts of the quadrotor controller separately, and plots the results.
clear; clc; close all;

%% Global parameters and controller gains
global params;
% Physical parameters
params = quadModel_readonly(); % Quad model

% Outer-loop (position) gains
params.kp_x = 1.0;  params.kp_y = 1.0;  params.kp_z = 1.5;
params.kd_x = 0.5;  params.kd_y = 0.5;  params.kd_z = 0.7;

% Inner-loop (attitude) gains
params.kp_phi   = 100;  params.kp_theta = 100;  params.kp_psi = 80;
params.kd_phi   = 10;   params.kd_theta = 10;   params.kd_psi = 8;

%% Simulation parameters for both tests
t_final = 10;       % total simulation time (seconds)
dt = 0.1;           % time step
t_vec = 0:dt:t_final;

%% ==================== Outer Loop Test ====================
% In this test, the current state is fixed (hover state with no rotation)
% while the desired position is stepped from [0;0;0] to [1;1;1] at t=2s.

% Preallocate storage for outputs
F_outer = zeros(size(t_vec));
M_outer = zeros(3, length(t_vec));

% Define a fixed state (hovering at the origin with no rotation)
% State vector s = [position(3); velocity(3); quaternion(4); angular velocity(3)]
s = [0; 0; 0;         % position: [0, 0, 0]
     0; 0; 0;         % velocity: [0, 0, 0]
     1; 0; 0; 0;       % attitude: identity quaternion (no rotation)
     0; 0; 0];        % angular velocity: [0, 0, 0]

for i = 1:length(t_vec)
    t = t_vec(i);
    % Desired state s_des = [pos_des; vel_des; acc_des; desired yaw; desired yaw rate]
    if t < 2
        pos_des = [0; 0; 0];
    else
        pos_des = [1; 1; 1];  % step to [1,1,1] at t=2s
    end
    s_des = [ pos_des;         % desired position
              0; 0; 0;         % desired velocity
              0; 0; 0;         % desired acceleration (feedforward = 0)
              0;               % desired yaw (0 rad)
              0];              % desired yaw rate
          
    [F_temp, M_temp] = controller(t, s, s_des);
    F_outer(i) = F_temp;
    M_outer(:, i) = M_temp;
end

% Plot outer loop outputs
figure;
subplot(2,1,1);
plot(t_vec, F_outer, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Thrust F (N)');
title('Outer Loop Test: Thrust Command vs Time');
grid on;

subplot(2,1,2);
plot(t_vec, M_outer(1,:), 'r', t_vec, M_outer(2,:), 'g', t_vec, M_outer(3,:), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Torques M (N*m)');
title('Outer Loop Test: Torque Commands vs Time');
legend('\tau_\phi','\tau_\theta','\tau_\psi');
grid on;

%% ==================== Inner Loop Test ====================
% In this test, we set the vehicle's attitude to a nonzero orientation
% (e.g., 10° roll, 5° pitch, 0° yaw) and then vary the desired yaw over time.
% This introduces an attitude error that the controller will try to correct.

% Preallocate storage for inner loop outputs
F_inner = zeros(size(t_vec));
M_inner = zeros(3, length(t_vec));

% Set the current state with a known attitude error.
% For example, let the Euler angles be: roll = 10°, pitch = 5°, yaw = 0°
phi0   = deg2rad(10);
theta0 = deg2rad(5);
psi0   = 0;
% Convert Euler angles to a quaternion [qw, qx, qy, qz]
qw = cos(phi0/2)*cos(theta0/2)*cos(psi0/2) + sin(phi0/2)*sin(theta0/2)*sin(psi0/2);
qx = sin(phi0/2)*cos(theta0/2)*cos(psi0/2) - cos(phi0/2)*sin(theta0/2)*sin(psi0/2);
qy = cos(phi0/2)*sin(theta0/2)*cos(psi0/2) + sin(phi0/2)*cos(theta0/2)*sin(psi0/2);
qz = cos(phi0/2)*cos(theta0/2)*sin(psi0/2) - sin(phi0/2)*sin(theta0/2)*cos(psi0/2);
quat = [qw; qx; qy; qz];

% Define a fixed state for inner loop test (position/velocity don't affect attitude control)
s_inner = [0; 0; 0;      % position
           0; 0; 0;      % velocity
           quat;         % attitude (nonzero roll & pitch)
           0; 0; 0];     % angular velocity (assumed zero for now)

for i = 1:length(t_vec)
    t = t_vec(i);
    % In this test, we hold desired position, velocity, and acceleration at zero.
    % But we let the desired yaw increase linearly to see how the controller reacts.
    psi_des = 0.1 * t;   % desired yaw (rad) increases with time
    s_des_inner = [0; 0; 0;    % desired position (no error)
                   0; 0; 0;    % desired velocity
                   0; 0; 0;    % desired acceleration
                   psi_des;    % desired yaw (time-varying)
                   0];         % desired yaw rate (0 for simplicity)
               
    [F_temp, M_temp] = controller(t, s_inner, s_des_inner);
    F_inner(i) = F_temp;
    M_inner(:, i) = M_temp;
end

% Plot inner loop outputs
figure;
subplot(2,1,1);
plot(t_vec, F_inner, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Thrust F (N)');
title('Inner Loop Test: Thrust Command vs Time');
grid on;

subplot(2,1,2);
plot(t_vec, M_inner(1,:), 'r', t_vec, M_inner(2,:), 'g', t_vec, M_inner(3,:), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Torques M (N*m)');
title('Inner Loop Test: Torque Commands vs Time');
legend('\tau_\phi','\tau_\theta','\tau_\psi');
grid on;

% Optionally, plot desired yaw vs. current yaw for inner loop test.
% (Here, current yaw is extracted from s_inner and remains constant.)
[~, ~, psi_actual] = quatToEulerZYX(s_inner(7:10));
figure;
plot(t_vec, 0.1*t_vec, 'k--', t_vec, psi_actual*ones(size(t_vec)), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Yaw (rad)');
legend('Desired Yaw','Actual Yaw');
title('Inner Loop Test: Yaw Comparison');
grid on;


