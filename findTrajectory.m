function [is_valid, opt_time, sumTorques] = findTrajectory(th_start, th_end, dth_start, dth_end, plotting, from_python)
    global positions 
    global velocities
    global accelerations
    global torques
    % clf;
    % Add all subfolders to the path (need to include MR package)
    addpath(genpath(pwd))

    %% Import the arm setup
    alphaArm = alphaSetup();
    % homog T of end effector in the home configuration
    M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];
    
    if from_python == 1
        [th_start, th_end, dth_start, dth_end] = fixinputtypes(th_start, th_end, dth_start, dth_end);
    end
    
    is_valid = 1; % start assuming it's valid
    opt_time = 1;
    sumTorques = 0;
    

    %% ---------- Dynamics ----------
    g = [0; 0; 9.807]; % in m/s2
    % No force at the end effector
    Ftip = [0; 0; 0; 0; 0; 0];
    th1mth2 = th_start - th_end;
    curr_theta = th_start; curr_dtheta = dth_start;
    ddtheta0 = [0; 0; 0; 0; 0];

    iterations = 20;    
    T = 1;
    disp('Finding intial trajectory')
    [traj1poses, traj1vels] = find_path(T, iterations, th_start, th_end, dth_start, dth_end);
      
    is_valid = check_angle_limits(traj1poses)
    are_vels_valid = check_vel_limits(traj1vels)
    
    if is_valid == 1 && are_vels_valid == 1
        find_cost(T, iterations, th_start, th_end, dth_start, dth_end)
        sumTorques = sum(abs(torques*T/iterations), 'all');
    elseif is_valid == 1 && are_vels_valid == 0
        disp('Path is valid, but vels are not. Iterating.')
        while are_vels_valid == 0 && T < 10.1
%             disp('.')
            T = T+.05;
            iterations = iterations+1;
            [~, next_vels] = find_path(T, iterations, th_start, th_end, dth_start, dth_end);
            are_vels_valid = check_vel_limits(next_vels);
        end
        if are_vels_valid == 1
            disp('Velocities in valid range. Time scaling:')
            disp(T)
            find_cost(T, iterations, th_start, th_end, dth_start, dth_end)
            sumTorques = sum(abs(torques*T/iterations), 'all');
            opt_time = T;
        else 
            disp('Iteration done, no time scaling found. Reporting invalid path.')
            is_valid = 0;
            sumTorques = 1000;
            opt_time = T;
        end
    else 
        disp('Invalid path. Reporting high torque.')
        is_valid = 0;
        sumTorques = 1000;
        opt_time = T;
    end
    
    
    %% ---------- Plotting ----------
    if plotting == 1
        % Show the arm graphically
%         figure
%         alphaArm.plot(traj1poses.', 'jointdiam', 1.5, 'jvec', 'nobase');
%         hold on
% 
%         % plot the base in the correct orientation
%         [X, Y, Z] = cylinder(.020);
%         surf(Z*.25, Y, X, 'FaceColor', 'k');
        
        if is_valid == 0
            timestep = 0:1:20;
        else
            timestep = 0:T/iterations:T;
        end
        length(timestep)
        length(traj1poses(1,:))

        figure 
        plot(timestep, traj1poses(1,:))
        hold on
        plot(timestep, traj1poses(2,:))
        plot(timestep, traj1poses(3,:))
        plot(timestep, traj1poses(4,:))
        xlabel('Time (steps)')
        ylabel('Joint Angle (theta)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

        figure 
        plot(timestep, traj1vels(1,:))
        hold on
        plot(timestep, traj1vels(2,:))
        plot(timestep, traj1vels(3,:))
        plot(timestep, traj1vels(4,:))
        xlabel('Time (steps)')
        ylabel('Joint Velocities (d_theta)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

%         figure 
%         plot(0:dt:dt*iterations, accelerations(1,:))
%         hold on
%         plot(0:dt:dt*iterations, accelerations(2,:))
%         plot(0:dt:dt*iterations, accelerations(3,:))
%         plot(0:dt:dt*iterations, accelerations(4,:))
%         xlabel('Time (s)')
%         ylabel('Joint Accelerations (d_d_theta)') 
%         legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

        if is_valid == 1
            figure 
            plot(timestep, torques(1,:))
            hold on
            plot(timestep, torques(2,:))
            plot(timestep, torques(3,:))
            plot(timestep, torques(4,:))
            xlabel('Time (s)')
            ylabel('Torque (Nm)') 
            legend('Joint E', 'Joint D', 'Joint C', 'Joint B')
        end
    end

    %% ---------- Dynamics iterator ---------
    function [theta_new, dtheta_new, ddtheta_new, taulist] = step_dynamics_forward(thetalist, dthetalist, ddthetalist, Ftip, g, dt)
        [M, RHS, taulist] = closedFormInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip, g);
        dqdt = dthetalist;
        dqdot_dt = M\(taulist - RHS);

        theta_new = thetalist + dqdt*dt;
        dtheta_new = dthetalist + dqdot_dt*dt;
        ddtheta_new = dqdot_dt;
    end

    %% ---------- Type parser for Python input ----------
    function [parsed_th1, parsed_th2, parsed_dth1, parsed_dth2] = fixinputtypes(th_start, th_end, dth_start, dth_end)
        parsed_th1 = transpose(cell2mat(th_start));
        parsed_th2 = transpose(cell2mat(th_end));
        parsed_dth1 = transpose(cell2mat(dth_start));
        parsed_dth2 = transpose(cell2mat(dth_end));
    end

    function reset_vectors(iterations, th_start, dth_start)
        ddtheta0 = [0; 0; 0; 0; 0];
        positions = zeros([5, iterations]);
        velocities = zeros([5, iterations]);
        accelerations = zeros([5, iterations]);
        torques = zeros([5, iterations]);
        positions(:,1) = th_start;
        velocities(:,1) = dth_start;
        accelerations(:,1) = ddtheta0; 
    end

    function angles_are_valid = check_angle_limits(thetas)
        angles_are_valid = 1; % assume angles are valid
        theta_limits = [-175, 175; ...
        -74.61, 125.89; ...
        -164.61, 35.39; ...
        -165, 165; ...
        -180, 180]*pi/180;

        for j = 1:4
%             disp('Checking joint')
%             disp(j)
            if all(thetas(j,:) > theta_limits(j,1)) && all(thetas(j,:) < theta_limits(j,2))
%                 disp('Theta within limits')
            else
%                 disp('Theta not within limits')
                angles_are_valid = 0;   
            end
        end
    end

    function vels_are_valid = check_vel_limits(vels)
        vels_are_valid = 1;
        theta_dot_limits = [-30, 30; ...
            -30, 30; ...
            -30, 30; ...
            -50, 50; ...
            -50, 50]*pi/180;
        
        for k = 1:4
%             disp('Checking joint velocities')
%             disp(k)
            if all(vels(k,:) > theta_dot_limits(k,1)) && all(vels(k,:) < theta_dot_limits(k,2))
%                 disp('ThetaDot within limits')
            else
%                 disp('ThetaDot not within limits')
%                 disp(k)
                vels_are_valid = 0;
            end
        end
    end

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

    function find_cost(T, iterations, th_start, th_end, dth_start, dth_end)
            curr_theta = th_start; curr_dtheta = dth_start;
            reset_vectors(iterations, th_start, dth_start)
            [~, ~, init_torques] = closedFormInverseDynamics(5, th_start, dth_start, ddtheta0, Ftip, g);
            torques(:,1) = init_torques;
            dt2 = T/iterations;

            for i = 1:iterations
                t = dt2*i;
                curr_ddtheta = 6*-2*(5*th1mth2+3*T*dth_start+2*T*dth_end)*t/T^3 + ...
                    12*(15*th1mth2+8*T*dth_start + 7*T*dth_end)*t^2/T^4 + ...
                    20*-3*(2*th1mth2+ T*dth_start + T*dth_end)*t^3/T^5;
                [theta_new, dtheta_new, ~, taulist] = step_dynamics_forward(curr_theta, curr_dtheta, curr_ddtheta, Ftip, g, dt2);
                curr_theta = theta_new; curr_dtheta = dtheta_new; 
                torques(:,i+1) = taulist;
                positions(:,i+1) = theta_new;
                velocities(:,i+1) = dtheta_new;
                accelerations(:,i+1) = curr_ddtheta;
            end
    end
end

