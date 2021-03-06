% Runs the findTrajectory script
%         theta_limits = [-175, 175; ...
%         -74.61, 125.89; ...
%         -164.61, 35.39; ...
%         -165, 165; ...
%         -180, 180]*pi/180;
th1 = [2.84,-1.79,-2.81,-2.88,0.0].';
dth1 = [0.0,0.0,0.0,0.0,0.0].';
th2 = [0.14,-1.63,-0.21,-2.88, 0.0].';
dth2 = [0.34,0.27,0.4,0.38, 0.0].';
th3 = [-2.18,-1.74,0.28,-2.88, 0.0].';
dth3 = [0.15,0.49,0.21,0.55,0.0].';
th4 = [0.72,-1.54,-1.95,-2.88,0.0].';
dth4 = [0.34,0.27,0.4,0.38, 0.0].';
th5 = [2.27,-1.99,0.06,-2.88,0.0].';
dth5 = [0.29,0.17,0.41,0.45, 0.0].';
th6 = [-0.3,-1.53,-0.36,-2.88, 0.0].';
dth6 = [0.0,0.0,0.0,0.0, 0.0].';

thetas = [th1 th2 th3 th4 th5 th6];% th4 th5 th6];
dthetas = [dth1 dth2 dth3 dth4 dth5 dth6];% dth4 dth5 dth6];
% theta_1 = [0 100 0 0 0].'*pi/180;
% theta_2 = [10 -20 -40 10 0].'*pi/180;
% dtheta_1 = [0 0 0 0 0].'*pi/180;
% dtheta_2 = [5 -10 -15 -20 0].'*pi/180;

% th1, th2, dth1, dth2, plotting, from_python
times = [];
sum_torque = 0;
all_waypoints = [];
for i = 1:length(thetas(1,:))-1
    disp('Trajectory number:')
    disp(i)
    [is_valid, opt_time, sumTorques] = findTrajectory(thetas(:,i), thetas(:,1+1), dthetas(:,i), dthetas(:,i+1), 0, 0)
    times = [times opt_time];
    sum_torque = sum_torque + sumTorques;
    [poses, ~] = find_path(opt_time, 50, thetas(:,i), thetas(:,1+1), dthetas(:,i), dthetas(:,i+1));
    all_waypoints = [all_waypoints poses];
end
alphaArm = alphaSetup();
alphaArm.plot(all_waypoints.', 'jointdiam', 1.5, 'jvec', 'nobase');


    function [est_poses, est_vels] = find_path(T, iterations, th_start, th_end, dth_start, dth_end)
        dt1 = T/iterations;
        est_vels = zeros([5, iterations]);
        est_vels(:,1) = dth_start;
        est_poses = zeros([5, iterations]);
        est_poses(:,1) = th_start;

        for i = 1:iterations
            t = dt1*i;
            est_theta = th_start + dth_start*t - (3*t^5*(2*th_start - 2*th_end + T*dth_start + T*dth_end))/T^5 - ...
                (2*t^3*(5*th_start - 5*th_end + 3*T*dth_start + 2*T*dth_end))/T^3 + ...
                (t^4*(15*th_start - 15*th_end + 8*T*dth_start + 7*T*dth_end))/T^4;
            est_poses(:, i+1) = est_theta;
            est_dtheta = dth_start - (15*t^4*(2*th_start - 2*th_end + T*dth_start + T*dth_end))/T^5 - ...
                (6*t^2*(5*th_start - 5*th_end + 3*T*dth_start + 2*T*dth_end))/T^3 + ...
                (4*t^3*(15*th_start - 15*th_end + 8*T*dth_start + 7*T*dth_end))/T^4;
            est_vels(:, i+1) = est_dtheta;
        end
    end

