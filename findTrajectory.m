function [positions, velocities, sumTorques] = findTrajectory(th_start, th_end, dth_start, dth_end, plotting)

    % clf;
    % Add all subfolders to the path (need to include MR package)
    addpath(genpath(pwd))

    %% Import the arm setup
    alphaArm = alphaSetup();
    % homog T of end effector in the home configuration
    M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];


    %% ---------- Dynamics ----------
    g = [0; 0; 9.807]; % in m/s2
    % 0 acceleration to start
    ddtheta0 = [0; 0; 0; 0; 0];
    % No force at the end effector
    Ftip = [0; 0; 0; 0; 0; 0];

    iterations = 50;
    positions = zeros([5, iterations]);
    positions(:,1) = th_start;
    velocities = zeros([5, iterations]);
    velocities(:,1) = dth_start;
    accelerations = zeros([5, iterations]);
    accelerations(:,1) = ddtheta0;
    torques = zeros([5, iterations]);
    [~, ~, init_torques] = closedFormInverseDynamics(5, th_start, dth_start, ddtheta0, Ftip, g);
    torques(:,1) = init_torques;
    
    T = 5;
    dt = T/iterations;
    th1mth2 = th_start - th_end;
    curr_theta = th_start; curr_dtheta = dth_start;
    for i = 1:iterations
        t = dt*i;
        curr_ddtheta = 6*-2*(5*th1mth2+3*T*dth_start+2*T*dth_end)*t/T^3 + ...
            12*(15*th1mth2+8*T*dth_start + 7*T*dth_end)*t^2/T^4 + ...
            20*-3*(2*th1mth2+ T*dth_start + T*dth_end)*t^3/T^5;
        [theta_new, dtheta_new, ~, taulist] = step_dynamics_forward(curr_theta, curr_dtheta, curr_ddtheta, Ftip, g, dt);
        curr_theta = theta_new; curr_dtheta = dtheta_new; 
        torques(:,i+1) = taulist;
        positions(:,i+1) = theta_new;
        velocities(:,i+1) = dtheta_new;
        accelerations(:,i+1) = curr_ddtheta;
    end
    
    sumTorques = sum(abs(torques*dt), 'all')


    %% ---------- Plotting ----------
    if plotting == 1
        % Show the arm graphically
        alphaArm.plot(positions.', 'jointdiam', 1.5, 'jvec', 'nobase');
        hold on

        % plot the base in the correct orientation
        [X, Y, Z] = cylinder(.020);
        surf(Z*.25, Y, X, 'FaceColor', 'k');

        figure 
        plot(0:dt:dt*iterations, positions(1,:))
        hold on
        plot(0:dt:dt*iterations, positions(2,:))
        plot(0:dt:dt*iterations, positions(3,:))
        plot(0:dt:dt*iterations, positions(4,:))
        xlabel('Time (s)')
        ylabel('Joint Angle (theta)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

        figure 
        plot(0:dt:dt*iterations, velocities(1,:))
        hold on
        plot(0:dt:dt*iterations, velocities(2,:))
        plot(0:dt:dt*iterations, velocities(3,:))
        plot(0:dt:dt*iterations, velocities(4,:))
        xlabel('Time (s)')
        ylabel('Joint Velocities (d_theta)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

        figure 
        plot(0:dt:dt*iterations, accelerations(1,:))
        hold on
        plot(0:dt:dt*iterations, accelerations(2,:))
        plot(0:dt:dt*iterations, accelerations(3,:))
        plot(0:dt:dt*iterations, accelerations(4,:))
        xlabel('Time (s)')
        ylabel('Joint Accelerations (d_d_theta)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')

        figure 
        plot(0:dt:dt*iterations, torques(1,:))
        hold on
        plot(0:dt:dt*iterations, torques(2,:))
        plot(0:dt:dt*iterations, torques(3,:))
        plot(0:dt:dt*iterations, torques(4,:))
        xlabel('Time (s)')
        ylabel('Torque (Nm)') 
        legend('Joint E', 'Joint D', 'Joint C', 'Joint B')
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

end
