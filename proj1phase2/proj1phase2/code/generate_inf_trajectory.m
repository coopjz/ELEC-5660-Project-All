% Parameters for the butterfly-like trajectory
A = 6.0;   % Amplitude along x-axis
B = 3.0;   % Amplitude along y-axis
C = 2;   % Amplitude along z-axis
omega = pi / 5;  % Frequency
T = linspace(0, 10, 40); % 20 discrete time steps

% Compute waypoints
path = zeros(length(T), 3);
for i = 1:length(T)
    t = T(i);
    path(i, 1) = A * sin(omega * t);         % x position
    path(i, 2) = B * sin(2 * omega * t);     % y position
    path(i, 3) = C * abs(sin(omega * t));    % z position (always positive)
end

% Display waypoints
disp('Generated butterfly-like waypoints:')
disp(path)

% Plot the trajectory
figure;
plot3(path(:,1), path(:,2), path(:,3), 'bo-', 'LineWidth', 1.5, 'MarkerSize', 5);
grid on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('Butterfly-like 3D Trajectory');
axis equal;
