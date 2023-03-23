%% clear all variables clear command window
close all %close all plots
%bdclose all %close all simulink block
clear all %clear all variables
clc %clear comman window

%% 3-D Path Tracing With Inverse Kinematics
%% Introduction
% This example shows how to create a simple 3D robot

%% Construct The Robot
% Create a |rigidBodyTree| object and rigid bodies with their
% associated joints. Specify the geometric properties of each rigid body
% and add it to the robot.

%% 
% Start with a blank rigid body tree model.
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
%%
% Specify arm lengths for the robot arm.
L1 = 0.6;
L2 = 0.4;
L3 = 0.4;
%%
% Add |'link1'| body with |'joint1'| joint.
body = rigidBody('link1');
%%% body definition
body.Mass=1; %1 kg (default)
body.CenterOfMass=[0 0 0]; % [0 0 0] m (default) | [x y z] vector
body.Inertia=[1 1 1 0 0 0]; % [1 1 1 0 0 0] kg.m2 (default) | [Ixx Iyy Izz Iyz Ixz Ixy] vector
%%% Joint definition
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
%%
% Add |'link2'| body with |'joint2'| joint.
body = rigidBody('link2');
%%% body definition
body.Mass=1; %1 kg (default)
body.CenterOfMass=[0 0 0]; % [0 0 0] m (default) | [x y z] vector
body.Inertia=[1 1 1 0 0 0]; % [1 1 1 0 0 0] kg.m2 (default) | [Ixx Iyy Izz Iyz Ixz Ixy] vector
%%% Joint definition
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,0,L1]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link1');
%%
% Add |'link3'| body with |'joint2'| joint.
body = rigidBody('link3');
%%% body definition
body.Mass=1; %1 kg (default)
body.CenterOfMass=[0 0 0]; % [0 0 0] m (default) | [x y z] vector
body.Inertia=[1 1 1 0 0 0]; % [1 1 1 0 0 0] kg.m2 (default) | [Ixx Iyy Izz Iyz Ixz Ixy] vector
%%% Joint definition
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link2');
%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = rigidBody('tool');
%%% body definition
body.Mass=1; %1 kg (default)
body.CenterOfMass=[0 0 0]; % [0 0 0] m (default) | [x y z] vector
body.Inertia=[1 1 1 0 0 0]; % [1 1 1 0 0 0] kg.m2 (default) | [Ixx Iyy Izz Iyz Ixz Ixy] vector
%%% Joint definition
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');

%%
% Show details of the robot to validate the input properties. 
% The robot should have two non-fixed joints for the rigid bodies and a fixed body for the end-effector.
% default values:
% robot.Gravity=[0 0 -9.81] %[0 0 0] (default). i.e. no gravity. To change use: robot.Gravity=[0 0 -9.81]

robot %rigidBodyTree with properties. 
showdetails(robot)

%figure(1); 
%show(robot) % show the robot config with zero initial conditions


%% open model
simmdl='WS_3DRobot_example' %simulink model file name
open(simmdl) %open simulink model

%% PID control definition
Kp1=100 %8*3
Kd1=10 %0.1*1
Kfilt1=150

Kp2=100 %8*3
Kd2=10 %0.1*1
Kfilt2=150

Kp3=100 %8*3
Kd3=10 %0.1*1
Kfilt3=150

%% Joint torques definition
% T1amp=1; %Joint 1 torque amplitude
% T2amp=1; %joint 2 torque amplitude
% T3amp=1; %joint 3 torque amplitude

%% Joint angles definition: to try for examples: [45 0], [0 45], [90 -90], [-90 90]
% jointangles=[45 0 0] %deg
% angle1ref=jointangles(1) %Desired joint 1 angle (deg)
% angle2ref=jointangles(2) %Desired joint 2 angle (deg)
% angle3ref=jointangles(3) %Desired joint 3 angle (deg)

%% end effector position definition
initPos=[0.6 0 0.3]; %initial position
initialConfig = homeConfiguration(robot); %inital joint angle

% eePos_des_sim=[0 0.6 0]; %desired position. Default [0 0.6 0];
% %targetPosition =  trvec2tform(eePos_des_sim) %desired position in homogeneous transformation

%% Traj generation
eePos_des_traj=[0.0 0.6 0.0 0.3;
    20.0 0.0 0.6 0.3;
    30.0 0.0 0.6 0.3;
    50.0 0.0 0.3 0.6
    55.0 0.0 0.3 0.6]
time_traj=eePos_des_traj(:,1)
eePos_des_traj=eePos_des_traj(:,2:4)

%% simulate model
tsim=55 %simulation time
t_sample=0.01 %0.01 %simulation time step (sample time)

sim(simmdl) %run the simulation of the model


%% simple plot of inputs and outputs
% figure(1)
% plot(t_sim,input_sim(:,1),'b', t_sim,input_sim(:,2),'--r')
% grid on; hold on;
% title('Input torques vs time'); ylabel('torque (N)'); xlabel('time (sec)')
% legend('torque 1',' torque 2')
% 
% figure(2)
% subplot(2,1,1); plot(t_sim,qs(:,1)*180/pi,'b', t_sim,qs_des(:,1)*180/pi,'--r'); grid on; hold on;
% legend('angle 1 (deg)',' angle 1 desired (deg)');
% ylabel('Angles (deg)'); xlabel('time (sec)');
% title('Joint angles vs time');
% subplot(2,1,2); plot(t_sim,qs(:,2)*180/pi,'b', t_sim,qs_des(:,2)*180/pi,'--r'); grid on; hold on;
% legend('angle 2 (deg)',' angle 2 desired(deg)')
% ylabel('Angles (deg)'); xlabel('time (sec)')
% 
% figure(3)
% subplot(2,1,1); plot(t_sim,eePos_sim(:,1),'b', t_sim,eePos_des(:,1),'--r'); grid on; hold on;
% title('End effector x position vs time'); ylabel('position (m)'); xlabel('time (sec)')
% legend('x','x desired')
% ylabel('x position'); xlabel('time (sec)')
% 
% subplot(2,1,2); plot(t_sim,eePos_sim(:,2),'b', t_sim,eePos_des(:,2),'--r'); grid on; hold on;
% title('End effector y position vs time'); ylabel('position (m)'); xlabel('time (sec)')
% legend('y','y desired')
% ylabel('y position'); xlabel('time (sec)')

% subplot(3,1,3); plot(t_sim,eePos_sim(:,3),'b', t_sim,eePos_des(:,3),'--r'); grid on; hold on;
% title('End effector z position vs time'); ylabel('position (m)'); xlabel('time (sec)')
% legend('z','z desired')
% ylabel('z position'); xlabel('time (sec)')



%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot configuration.
Robot_animate_3d
