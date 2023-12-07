%% TidyBot Matlab Code
clc; clear; close all;
import ETS3.*
import ETS2.*

%% Defining the robotic arm
L1 = 0.077; %%Base height
L2 = 0.24;  %
L3 = 0.024; %Joint length
L4 = 0.24;  %
L5 = 0.126; %Original 0.126 %Gripper length

%Calculating L6 and beta angle with pythagoras theroem
L6 = sqrt(L2*L2 + L3*L3);
beta = atan(L3/L2);

%% DH Parameters 
% | theta | d  | a | alfa | 
L(1) = Link('revolute', 'd', L1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', L6, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', L4, 'alpha', 0);
L(4) = Link('revolute', 'd', 0,  'a', L5, 'alpha', 0);
TidyBotArm = SerialLink(L,'name', 'TidyBot_Arm')

%Zero angle
TidyBotArm.plot([0, 0, 0, 0])

TidyBotArm.teach

%% Forward and inverse kinematics 1.1.2

theta = 0; theta2 = 0; theta3 = 0; theta4 = 0;         %deg2rad(theta)
d = L1; d2 = 0; d3 = 0; d4 = 0;
a = 0; a2 = L6; a3 = L4; a4 = L5;
alpha = deg2rad(90); alpha2= 0; alpha3 = 0; alpha4= 0; %deg2rad(theta)

%Figure av ønsket robot
TidyBotArm.plot([theta, theta2, theta3, theta4])
%Skriv også manuelt inn gradene så får du ønsket posisjon


%1
R_z_theta = [cos(theta) -sin(theta) 0 0; sin(theta), cos(theta), 0, 0; 0 0 1 0; 0 0 0 1];
T_z_d = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
T_x_a = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R_x_alpha = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];

T0_1 = R_z_theta * T_z_d * T_x_a * R_x_alpha;

%2
R_z_theta2 = [cos(theta2) -sin(theta2) 0 0; sin(theta2), cos(theta2), 0, 0; 0 0 1 0; 0 0 0 1];
T_z_d2 = [1 0 0 0; 0 1 0 0; 0 0 1 d2; 0 0 0 1];
T_x_a2 = [1 0 0 a2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R_x_alpha2 = [1 0 0 0; 0 cos(alpha2) -sin(alpha2) 0; 0 sin(alpha2) cos(alpha2) 0; 0 0 0 1];

T1_2 = R_z_theta2 * T_z_d2 * T_x_a2 * R_x_alpha2;

%3
R_z_theta3 = [cos(theta3) -sin(theta3) 0 0; sin(theta3), cos(theta3), 0, 0; 0 0 1 0; 0 0 0 1];
T_z_d3 = [1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
T_x_a3 = [1 0 0 a3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R_x_alpha3 = [1 0 0 0; 0 cos(alpha3) -sin(alpha3) 0; 0 sin(alpha3) cos(alpha3) 0; 0 0 0 1];

T2_3 = R_z_theta3 * T_z_d3 * T_x_a3 * R_x_alpha3;

%4
R_z_theta4 = [cos(theta4) -sin(theta4) 0 0; sin(theta4), cos(theta4), 0, 0; 0 0 1 0; 0 0 0 1];
T_z_d4 = [1 0 0 0; 0 1 0 0; 0 0 1 d4; 0 0 0 1];
T_x_a4 = [1 0 0 a4; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R_x_alpha4 = [1 0 0 0; 0 cos(alpha4) -sin(alpha4) 0; 0 sin(alpha4) cos(alpha4) 0; 0 0 0 1];

T3_4 = R_z_theta4 * T_z_d4 * T_x_a4 * R_x_alpha4;

%Forward kinematics manual
T_0_4 = T0_1 * T1_2 * T2_3 * T3_4

%Forward Kinematics from the specified angles theta1,2,3,4
%T_0_4_dh = TidyBotArm.fkine([0 0 0 0])
T_0_4_dh_Toy_Pickup = TidyBotArm.fkine([deg2rad(0) deg2rad(0) deg2rad(-54) deg2rad(54)])
%Inverse Kinematics
q_0_4_InverseKinematics = TidyBotArm.ikine(T_0_4_dh_Toy_Pickup, 'mask', [1 1 1 0 0 0])

%% Homogenous Transformation 
%TidyBotArm.teach

%  Base --> End-effector positions xyz Resting position
B_Pose_E = [0.364 0 0.008];
B_Transform_E = rt2tr(eye(3), B_Pose_E)

% B --> C positions XYZ
B_Pose_C = [0.364 0 0.008];
B_Transform_C = rt2tr(eye(3), B_Pose_C)

% Camera --> Toy position XYZ
C_Pose_T = [0.144 0 -0.125];
C_Transform_T = rt2tr(eye(3), C_Pose_T)

% Transformation from Base to Toy
B_Transform_T = B_Transform_C*C_Transform_T

% Homogenous transformation to the toy at XYZ = [0.508 0 -0.117]
Q_Toy = TidyBotArm.ikine(B_Transform_T,'mask',[1 1 1 0 0 0])

TidyBotArm.plot(Q_Toy)
hold on

%Visuals
%Plotting base location in simulation
trplot(eye(3),'color','b','frame','B')

%Plotting resting position Camera in simulation
trplot(B_Transform_C,'color','r','frame','C')

%Plotting toy position in simulation
trplot(B_Transform_T,'color','g','frame','T')

TidyBotArm.plot(Q_Toy)
grid on

% %% Modelling version 1 (Using jtraj)
% 
% q0 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position
% q1 = [deg2rad(0) deg2rad(32.4) deg2rad(-111.6) deg2rad(46.8)]; % Engange pick up of toy underneath sofa
% q2 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(61.2)];    % Don't crash with the sofa
% q3 = [deg2rad(0) deg2rad(0) deg2rad(-54) deg2rad(54)];         % Pick up toy
% q3 = Q_Toy  % Doesn't look good and crashed with sofa
% q4 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(54)];      % Get out from the sofa with toy in gripper
% q5 = [deg2rad(0) deg2rad(28.8) deg2rad(-115.2) deg2rad(54)];   % Don't crash with the sofa
% q6 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position -> Drive toy to wished position
% 
% % Trajectories Modelling and testing
% steg = 45;
% qi = jtraj(q0, q1, steg);
% qii = jtraj(q1, q2, steg);
% qiii = jtraj(q2, q3, steg);
% qiiii = jtraj(q3, q4, steg);
% qiiiii = jtraj(q4, q5, steg);
% qiiiiii = jtraj(q5, q6, steg);
% 
% % Trajectories Matrix
% QiMatrix = [qi; qii; qiii; qiiii; qiiiii; qiiiiii];
% 
% %Plotter QiMatrix
% TidyBotArm.plot(QiMatrix, 'trail', 'r');

%% Modelling version 2 (Finale) using mtraj which is smoother velocity-wise.
% Requirments: 1.A + 1.D Robot arm capable of picking toys from the floor + Under Sofa
% Simulate the kinematics of your robot in Matlab 3.1 Motion planning

q0 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position
q1 = [deg2rad(0) deg2rad(32.4) deg2rad(-111.6) deg2rad(46.8)]; % Engange pick up of toy underneath sofa
q2 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(61.2)];    % Don't crash with the sofa, drive forward with base
q3 = [deg2rad(0) deg2rad(0) deg2rad(-54) deg2rad(54)];         % Pick up toy
%q3 = Q_Toy  % Doesn't look good and crashed with sofa
q4 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(54)];      % Get out from the sofa with toy in gripper, more decline angle
q5 = [deg2rad(0) deg2rad(28.8) deg2rad(-115.2) deg2rad(54)];   % Don't crash with the sofa
q6 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position -> Drive toy to wished position

% Trajectories Modelling and testing
steg = 45;
qi = mtraj(@lspb,q0(end,:), q1(1,:), steg);
qii = mtraj(@lspb,q1(end,:), q2(1,:), steg);
qiii = mtraj(@lspb,q2(end,:), q3(1,:), steg);
qiiii = mtraj(@lspb,q3(end,:), q4(1,:), steg);
qiiiii = mtraj(@lspb,q4(end,:), q5(1,:), steg);
qiiiiii = mtraj(@lspb,q5(end,:), q6(1,:), steg);

% Trajectories Matrix
QiMatrix = [qi; qii; qiii; qiiii; qiiiii; qiiiiii];

%Plotter QiMatrix
TidyBotArm.plot(QiMatrix, 'trail', 'r');

%% Differential kinematics Force = 300g + payload Max. 500g = 0.8 kg

force = [ 0 0.8*0.981 0 0 0 0]                      %Max 0.8kg
Jacobian_force = TidyBotArm.jacobe(q3)' * force';   %Force on endEffector. Jacobo = Force on base
Jacobian_forceNewton = Jacobian_force               %Force on Joint1,2,3,4.


