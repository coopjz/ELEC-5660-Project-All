function s_des = square_trajectory(t, true_s)
    s_des = zeros(11,1);
% Given yaw, DO NOT CHANGE
    yaw_des = mod(0.2 * pi * t,2 * pi);
    dyaw_des = 0.2 * pi;
    
% TODO: Implement square trajectory here
    omega=25;

%square length
    L = 2;          
    T_total = 25;    
    T_seg = T_total / 4;  % Time to traverse one side
    t_mod = mod(t, T_total);  % Time within the current square cycle

    corners = [ 0,  0, 0;
                1,  2, 0;
                2,  2, 2;
                3,  0, 2;
                4,  0, 0];
    seg_idx = floor(t_mod / T_seg) + 1;   
    t_seg = mod(t_mod, T_seg); 
    start_pt = corners(seg_idx, :);
    end_pt = corners(seg_idx+1, :);
    delta = end_pt - start_pt;
    D = norm(delta);
    t_acc = T_seg/4;         % acceleration duration
    t_dec_start = T_seg*3/4;   

    if D > 0
        a = D / (t_acc * (T_seg - t_acc));    % constant acceleration magnitude
        V_max = a * t_acc;                      
    else
        a = 0;
        V_max = 0;
    end

    if t_seg <= t_acc
        % Acceleration 
        pos_along = 0.5 * a * t_seg^2;
        vel_along = a * t_seg;
        acc_along = a;
    elseif t_seg <= t_dec_start
        % Constant velocity
        pos_along = 0.5 * a * t_acc^2 + V_max * (t_seg - t_acc);
        vel_along = V_max;
        acc_along = 0;
    else
        % Deceleration 
        t_dec = t_seg - t_dec_start;
       
        pos_before_dec = 0.5 * a * t_acc^2 + V_max * (t_dec_start - t_acc);
        % Deceleration phase: 
        pos_along = pos_before_dec + V_max * t_dec - 0.5 * a * t_dec^2;
        vel_along = V_max - a * t_dec;
        acc_along = -a;
    end
    if D > 0
        u = delta / D;  % Unit vector along the segment
    else
        u = [0, 0, 0];
    end
    pos = start_pt + pos_along * u;
    vel = vel_along * u;
    acc = acc_along * u;
%     ratio = t_seg / T_seg;
%     pos = corners(seg_idx, :) + ratio * (corners(seg_idx+1, :) - corners(seg_idx, :));
%     vel = (corners(seg_idx+1, :) - corners(seg_idx, :)) / T_seg;
    x_des = pos(1);
    y_des = pos(2);
    z_des = pos(3);
    x_vdes= vel(1);
    y_vdes = vel(2);
    z_vdes =vel(3);
    x_ades =acc(1);
    y_ades=acc(2);
    z_ades=acc(3);

    s_des(1)=x_des; 
    s_des(2)=y_des; 
    s_des(3)=z_des; 
    s_des(4)=x_vdes; 
    s_des(5)=y_vdes; 
    s_des(6)=z_vdes;
    s_des(7)=x_ades; 
    s_des(8)=y_ades; 
    s_des(9)=z_ades;
    s_des(10)=yaw_des; 
    s_des(11)=dyaw_des; 
    
end
