
function [alpha_joint_frames, alpha_link_frames, MlistForward, MlistBackward, Slist, Alist, Glist, GpAlist] = AlphaKinematics()

%% ---------- SETUP ----------
    alphaArm = alphaSetup();
    Link1 = alphaArm.links(1);
    Link2 = alphaArm.links(2);
    Link3 = alphaArm.links(3);
    Link4 = alphaArm.links(4);
    Link5 = alphaArm.links(5);
    
%% ---------- TWISTS ---------
    % Twists calculated by hand by Hannah 01/2021
    % Format: [w, v]
    Slist = [[0; 0; 1; 0; 0; 0], ...
        [0; 1; 0; -.0462; 0; -0.02], ...
        [0; -1; 0; .0462; 0; 0.1707], ...
        [1; 0; 0; 0; 0.0262; 0], ...
        [0; 0; -1; 0; -0.3507; 0]];

%% ---------- HOMOGENEOUS TRANSFORMS (JOINTS) ----------
    
    QspaceStraight =[0 0 0 0 0];
    [~, all] = alphaArm.fkine(QspaceStraight);
    
    Tnaught = SE3();
    T_0e = SE3([[-1 0 0 0]; [0 -1 0 0]; [0 0 1 0]; [0 0 0 1]]);
    T_0d = SE3(all(1));
    T_0c = SE3(all(2));
    T_0b = SE3(all(3));
    T_0a = SE3(all(4));
    T_0ee = SE3(all(5));
    
    alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
    
%% ---------- HOMOGENEOUS TRANSFORMS (LINKS) ----------

R0 = rpy2r([0 0 0]);     
    T_link1_from_jointE = SE3(R0, Link1.r);
    T_0_L1 = SE3(T_0e.T*T_link1_from_jointE.T);
    
    T_link2_from_jointD = SE3(R0, Link2.r);
    T_0_L2 = SE3(T_0d.T*T_link2_from_jointD.T);
    
    T_link3_from_jointC = SE3(R0, Link3.r);
    T_0_L3 = SE3(T_0c.T*T_link3_from_jointC.T);
    
    T_link4_from_jointB = SE3(R0, Link4.r);
    T_0_L4 = SE3(T_0b.T*T_link4_from_jointB.T);
    
    T_link5_from_jointA = SE3(R0, Link5.r); % jaw1 x=-10, jaw2 x=-10; both y = -45
    T_0_L5 = SE3(T_0a.T*T_link5_from_jointA.T);
    
%     jaw_disp = 10/1000.0; % 10mm away from COM of hand
%     
%     T_ee_from_L5 = SE3(R0, [0, -jaw_disp, 0]);
%     T_0_ee = SE3(T_0_L5.T*T_ee_from_L5.T);
    alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_L5, T_0ee];
   
    num_link_frames = length(alpha_link_frames);
    inv_a_link_frames = [];
    for i = 1:num_link_frames
        inv_a_link_frames = [inv_a_link_frames SE3(inv(alpha_link_frames(i).T))];
    end  
    
    % inv_a_link_frames = [Tnaught, T_L1_0, T_L2_0, T_L3_0, T_L4_0, T_L5_0, T_ee_0]
    
%% ---------- RELATIVE LINK FRAMES (M) ----------
    M_backward = [];
    M_forward = [];
    
    for j = 2:num_link_frames
        M_j_jminus1 = SE3(inv_a_link_frames(j).T*alpha_link_frames(j-1).T);
        M_backward = [M_backward M_j_jminus1];
        
        M_jminus1_j = SE3(inv_a_link_frames(j-1).T*alpha_link_frames(j).T);
        M_forward = [M_forward M_jminus1_j];
    end

    % M_0_L1, M_L1_L2, M_L2_L3, M_L3_L4, M_L4_L5, M_L5_ee
    MlistForward = cat(3, M_forward(1).T, M_forward(2).T, M_forward(3).T, M_forward(4).T, M_forward(5).T, M_forward(6).T);
    MlistBackward = cat(3, M_backward(1).T, M_backward(2).T, M_backward(3).T, M_backward(4).T, M_backward(5).T, M_backward(6).T);
    
%% ---------- TWISTS IN LINK FRAMES (Ai) ----------
    Alist = [];
    for i = 1:5
        S_i = Slist(:,i);
        A_i = Adjoint(inv_a_link_frames(i+1).T)*S_i;
        Alist = [Alist A_i];
    end
 
%% ---------- Spatial Inertia Matrices Gi ----------
    Gi_matrices = {};
    Gi_matrices_plus = {};
    
%    % Added Mass terms
%    % Volume for Added Mass of Link 2 (and Link 4)
%    V_al2 = pi*.02^2*.15;
%    V_al4 = pi*.02^2*.22;
%    
%    m_al2 = V_al2*1002.7; %density of fresh water
%    m_al4 = V_al4*1002.7;
%    added_masses = [0 m_al2 0 m_al4 0];
%    
%    % Added inertias
%    AL2_Ixx = 1/2*m_al2*0.02^2;
%    AL4_Ixx = 1/2*m_al4*0.02^2;
%    AL2_I = 1/12*m_al2*0.02^2;
%    AL4_I = 1/12*m_al4*0.02^2;
%    added_inertias = [0 0 0; ...
%        AL2_Ixx AL2_I AL2_I; ...
%        0 0 0; 
%        AL4_Ixx AL4_I AL4_I; 
%        0 0 0];
    
    for i = 1:5
        link = alphaArm.links(i);
        m = link.m;
        G_i = [link.I(1,:) 0 0 0; link.I(2,:) 0 0 0; link.I(3,:) 0 0 0; ...
            0 0 0 m 0 0; 0 0 0 0 m 0; 0 0 0 0 0 m;];
        Gi_matrices{i} = G_i;
        
%         G_i_added_mass = [added_inertias(i,1) 0 0 0 0 0; 0 added_inertias(i,2) 0 0 0 0; 0 0 added_inertias(i,3) 0 0 0; ...
%             0 0 0 added_masses(i) 0 0; 0 0 0 0 added_masses(i) 0; 0 0 0 0 0 added_masses(i)];
%         Gi_matrices_plus{i} = G_i + G_i_added_mass;
    end
    
    AM1 = diag([7*10^-6     7*10^-6     0           .032    .032    .017]);
    AM2 = diag([0           1716*10^-6  1716*10^-6  .017    .201    .201]);
    AM3 = diag([7*10^-6     0           7*10^-6     .032    .017    .032]);
    AM4 = diag([2443*10^-6  2443*10^-6  0           .226    .226    .017]);
    
    Glist = cat(3, Gi_matrices{1}, Gi_matrices{2}, Gi_matrices{3}, Gi_matrices{4}, Gi_matrices{5});
    GpAlist = cat(3,Gi_matrices{1}+AM1, Gi_matrices{2}+AM2, Gi_matrices{3}+AM3, Gi_matrices{4}+AM4, Gi_matrices{5});
end