function s_des = hover_yaw(t, true_s)
    
    yaw_des = pi/6;
    dyaw_des = 0;
    s_des = zeros(11,1);
    s_des(10)=yaw_des;
    s_des(11)=dyaw_des;

end

