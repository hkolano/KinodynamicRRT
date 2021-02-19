% Runs the findTrajectory script
theta_1 = [0 0 0 0 0].'*pi/180;
theta_2 = [-45 -70 -100 10 0].'*pi/180;
dtheta_1 = [0 0 0 0 0].'*pi/180;
dtheta_2 = [5 -10 -15 -20 0].'*pi/180;

[outputThetas, outputThetaDots, sumTorques] = findTrajectory(theta_1, theta_2, dtheta_1, dtheta_2, 1);
