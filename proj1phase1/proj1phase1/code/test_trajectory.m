% Used for HKUST ELEC 5660

close all;
clc;
clear all;
addpath('./utils','./readonly');



figure(1)
h1 = subplot(3,6,[1,2,3,7,8,9,13,14,15]);
h2 = subplot(3,6,4);
h3 = subplot(3,6,5);
h4 = subplot(3,6,6);
h5 = subplot(3,6,10);
h6 = subplot(3,6,11);
h7 = subplot(3,6,12);
h8 = subplot(3,6,16);
h9 = subplot(3,6,17);
h10 = subplot(3,6,18);
set(gcf, 'Renderer', 'painters');
% Run Trajectory  three trajectories, test one by one
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, @hover_trajectory);


calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
calculate_RMSE_yaw=@(a,b) sqrt(mean(wrapToPi((a(:)-b(:))).^2));

global current_states
global desired_states
rmse_p = calculate_RMSE(current_states(:,1:3),desired_states(:,1:3));
rmse_v = calculate_RMSE(current_states(:,4:6),desired_states(:,4:6));
rmse_yaw = rad2deg(calculate_RMSE_yaw(current_states(:,7),desired_states(:,7)));

disp(['RMSE Position(m):',num2str(rmse_p)])
disp(['RMSE Velocity(m/s):',num2str(rmse_v)])
disp(['RMSE Yaw(deg):',num2str(rmse_yaw)])