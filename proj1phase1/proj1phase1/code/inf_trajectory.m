function s_des = inf_trajectory(t, true_s)
    % Initialize the desired state vector (11x1)
    s_des = zeros(11,1);
    
    %% Yaw (given, DO NOT CHANGE)
    yaw_des = mod(0.2 * pi * t, 2*pi);
    dyaw_des = 0.2 * pi;
    
    %% 
    A = 5;           % Amplitude 
    B = 5;           % Amplitude
    C=2;
    T_total = 20;     % Total period
    omega = 2*pi/T_total;
    
    %%
    x_des = A * sin(omega * t);
    y_des = B * sin(2 * omega * t);
    z_des = C * abs(sin(omega * t));     % Constant altitude (meters)
    
    %%
    x_vdes = A * omega * cos(omega * t);
    y_vdes = 2 * B * omega * cos(2 * omega * t);
    z_vdes = C * omega * cos(omega * t)*sign(sin(omega * t));
    
    %% Compute Accelerations (second derivative)
    x_ades = -A * omega^2 * sin(omega * t);
    y_ades = -4 * B * omega^2 * sin(2 * omega * t);
    z_ades = 0;
    
    %% Assemble the desired state vector
    s_des(1)  = x_des;    % x position
    s_des(2)  = y_des;   
    s_des(3)  = z_des;    
    s_des(4)  = x_vdes;   
    s_des(5)  = y_vdes;   
    s_des(6)  = z_vdes;  
    s_des(7)  = x_ades;   
    s_des(8)  = y_ades;  
    s_des(9)  = z_ades;  
    s_des(10) = yaw_des;  
    s_des(11) = dyaw_des; 
end