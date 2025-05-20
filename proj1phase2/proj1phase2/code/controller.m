function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;
maxF = params.maxF;
max_angle= params.maxangle;

% s(1:3) current position           
% s(4:6) current velocity
% s(7:10) current attitude quaternion
% s(11:13) current body angular velocity

% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate

cur_pos = s(1:3);
cur_vel = s(4:6);
q = s(7:10);
omega = s(11:13);

R = QuatToRot(q);
%row pitch   yaw
[phi,theta,psi]= RotToRPY_ZXY(R);

des_pos = s_des(1:3);
des_vel = s_des(4:6);
des_acc = s_des(7:9);
des_yaw = s_des(10);
des_yaw_rate = s_des(11);

%% PID position gains
% Kp_x =20; 
% Kp_y =20;
% Kp_z =15;
Kp_x =8.5; 
Kp_y =8.5;
Kp_z =36;
Kd_x =5.2;
Kd_y =5.2;
Kd_z =16.5;
Kp_pos = diag([8.5, 8.5,56]);
Kd_pos = diag([4.2, 5.2, 19.5]);
Ki_pos = diag([0.0005, 0.0005, 0.0005]);  % Integral gain


%% PID attitude gains (roll pitch yaw)
% Kp_phi =10;
% Kp_theta=10;
% Kp_psi =1;
% Kd_phi =0.15;
% Kd_theta=0.15;
% Kd_psi =0.1*Kp_psi;
Kp_phi =800;
Kp_theta=800;
Kp_psi =100;
Kd_phi =60;
Kd_theta=60;
Kd_psi =30;

Kp_att =diag([Kp_phi,Kp_theta,Kp_psi]);
Kd_att =diag([Kd_phi,Kd_theta,Kd_psi]);
max_yaw_rate = 2; % Maximum allowable yaw rate in rad/s


persistent pos_int_err  prev_t
    if isempty(pos_int_err)
        pos_int_err = zeros(3,1);
    end
    if isempty(prev_t)
        prev_t = t;
    end
    dt = max(t - prev_t, 1e-3);  % Ensure a minimum dt
    prev_t = t;

%% outer loop
pos_error = des_pos - cur_pos;
pos_int_err = pos_int_err + pos_error * dt;
%a_cmd -> P_dot_dot_cmd  gain calculation
a_cmd = des_acc + Kp_pos * (des_pos - cur_pos) + Kd_pos * (des_vel - cur_vel);

%force ->  m * ddot{x} = F * (R * e3) + m * [0;0;-g] thrust force
F_des = m * (a_cmd+ [0;0;g]);
u1 = F_des(3);
% calculate the angle
%body z axis
phi_c   = (1/g) * ( a_cmd(1)*sin(psi) - a_cmd(2)*cos(psi) );
theta_c = (1/g) * ( a_cmd(1)*cos(psi) + a_cmd(2)*sin(psi) );


phi_c = max(-max_angle, min(phi_c, max_angle));
theta_c= max(-max_angle, min(theta_c, max_angle));
% Desired yaw is given directly:
psi_c = des_yaw;
psi_c = clamp(psi_c);

%% inner loop
e_psi = psi_c - psi;
e_psi = clamp(e_psi); % Normalize the error to [-π, π]


e_att = [phi_c - phi; theta_c - theta; e_psi];
p_des = 0;
q_des = 0;
r_des =des_yaw_rate;

%r_des = sign(r_des) * min(abs(r_des), max_yaw_rate);


omega_des = [p_des; q_des; r_des];

e_omega = omega_des - omega;

tau = Kp_att * e_att + Kd_att * e_omega;
%cross_t =cross(omega,I*omega)    
u2 = I*tau +cross(omega,I*omega);

b3 = R(:, 3);
F =u1;
%F=dot(F_des,b3);
%F=max(min(F,maxF),-maxF);
M=u2;

end

function [result_angle] =  clamp(angle)
result_angle = atan2(sin(angle), cos(angle));
end

