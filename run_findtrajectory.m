% Runs the findTrajectory script
%         theta_limits = [-175, 175; ...
%         -74.61, 125.89; ...
%         -164.61, 35.39; ...
%         -165, 165; ...
%         -180, 180]*pi/180;
theta_1 = [0 0 0 0 0].'*pi/180;
theta_2 = [10 -20 -40 10 0].'*pi/180;
dtheta_1 = [0 0 0 0 0].'*pi/180;
dtheta_2 = [5 -10 -15 -20 0].'*pi/180;

% th1, th2, dth1, dth2, plotting, from_python
[is_valid, opt_time, sumTorques] = findTrajectory(theta_1, theta_2, dtheta_1, dtheta_2, 1, 0);


