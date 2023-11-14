%% TidyBot Matlab Code
clc; clear; close all;
import ETS3.*
import ETS2.*

%% Defining the robotic arm
L1 = 0.077; %Original 0.077 %Base height
L2 = 0.24; %Original 0.128 %1. Link
L3 = 0.024; %Original 0.024 % Joint
L4 = 0.24; %Original 0.124 %2. Link
L5 = 0.126; %Original 0.126 %Gripper length

%Calculating L6 and beta angle with pythagoras
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

q1 = 0; q2 = 0; q3 = 0; q4 = 0;
alpha1 = 1.5708; alpha234 = 0;
d1 = 0.077; d234 = 0;
a1 = 0; a2 = L6; a3 = L4; a4 = L5;

T_0_1 = [cos(q1) -sin(q1)*cos(alpha1) sin(q1)*cos(alpha1) a1*cos(q1);
         sin(q1) cos(q1)*cos(alpha1) -cos(q1)*sin(alpha1) alpha1*sin(q1);
         0       sin(alpha1)          cos(alpha1)         d1;
         0       0                    0                   1   ];

T_1_2 = [cos(q2) -sin(q2)*cos(alpha234) 0      a2*cos(q2);
         sin(q2) cos(q2)*cos(alpha234)  0      alpha234*sin(q2);
         0       0                    1      0;
         0       0                    0      1   ];
T_2_3 = [cos(q3) -sin(q3)*cos(alpha234) 0      a3*cos(q3);
         sin(q3) cos(q3)*cos(alpha234)  0      alpha234*sin(q3);
         0       0                    1      0;
         0       0                    0      1   ];

T_3_4 = [cos(q4) -sin(q4)*cos(alpha234) 0      a4*cos(q4);
         sin(q4) cos(q4)*cos(alpha234)  0      alpha234*sin(q4);
         0       0                    1      0;
         0       0                    0      1   ];

%Forward Kinematics from the specified angles Rest --> Pick Up --> Rest
T_0_4_dh = TidyBotArm.fkine([0 0 0 0])
%Inverse Kinematics
q_0_4_InverseKinematics = TidyBotArm.ikine(T_0_4_dh, 'mask', [1 1 1 0 0 0])
%Total matrix T0 to T4
T_0_4_manual = T_0_1 * T_1_2 * T_2_3 * T_3_4

%% Transformation 
%TidyBotArm.teach

%  Base --> End-effector positions xyz
B_Pose_E = [0.364 0 0.008];
B_Transform_E = rt2tr(eye(3), B_Pose_E)

% B --> C positions xyz
B_Pose_C = [0.364 0 0.008];
B_Transform_C = rt2tr(eye(3), B_Pose_C)

% Camera --> Toy position xyz
C_Pose_T = [0.508 0 -0.117];
C_Transform_T = rt2tr(eye(3), C_Pose_T)

% Transformation from Base to Toy
B_Transform_T = B_Transform_C*C_Transform_T

% Homogenous transformation to the toy
Q_Toy = TidyBotArm.ikine(B_Transform_T,'tol',1,'mask',[1 1 1 0 0 0])
%Does not work for me... Tried increasing tolerance, steps and iteration but no...

%Q_Toy = TidyBotArm.ikine(B_Transform_T,'ilimit',1000,'rlimit',2500,'mask',[1 1 1 0 0 0])
TidyBotArm.plot(Q_Toy)
hold on
%Visuals
trplot(eye(3),'color','b','frame','B')
trplot(B_Transform_C,'color','r','frame','C')
trplot(B_Transform_T,'color','g','frame','T')

TidyBotArm.plot(Q_Toy)
grid on

%% Modellering 
% Requirments: 1.A + 1.D Robot arm capable of picking toys from the floor + Under Sofa
% Simulate the kinematics of your robot in Matlab 3.1 Motion planning

q0 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position
q1 = [deg2rad(0) deg2rad(32.4) deg2rad(-111.6) deg2rad(46.8)]; % Engange pick up of toy underneath sofa
q2 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(61.2)];    % Don't crash with the sofa
q3 = [deg2rad(0) deg2rad(0) deg2rad(-54) deg2rad(54)];         % Pick up toy
q4 = [deg2rad(0) deg2rad(14.4) deg2rad(-90) deg2rad(54)];      % Get out from the sofa with toy in gripper
q5 = [deg2rad(0) deg2rad(28.8) deg2rad(-115.2) deg2rad(54)];   % Don't crash with the sofa
q6 = [deg2rad(0) deg2rad(50.4) deg2rad(-118.8) deg2rad(54)];   % Resting position -> Drive toy to wished position

% Trajectories Modelling and testing
steg = 45;
qi = jtraj(q0, q1, steg);
qii = jtraj(q1, q2, steg);
qiii = jtraj(q2, q3, steg);
qiiii = jtraj(q3, q4, steg);
qiiiii = jtraj(q4, q5, steg);
qiiiiii = jtraj(q5, q6, steg);

% Trajectories Matrix
QiMatrix = [qi; qii; qiii; qiiii; qiiiii; qiiiiii];

%Plotter QiMatrix
TidyBotArm.plot(QiMatrix, 'trail', 'r');

%% Differential kinematics Force = 300g + payload Max. 500g = 0.8 kg

force = [ 0 0.8*9.81 0 0 0 0]                       %Max 0.8kg
Jacobian_force = TidyBotArm.jacobe(q3)' * force';        %Kraft i endEffector. JacobO = kraft i basen
Jacobian_forceNewton = Jacobian_force               %Force on Joint1,2,3,4.
