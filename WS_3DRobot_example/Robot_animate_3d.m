%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.

%%
% Show the robot in the first configuration of the trajectory. 
% Adjust the plot to show the 3-D shape is drawn on. 
% Plot the desired vs actual trajectories.
figure
show(robot,qs(1,:)');
%view(2) %2D view
view(3) %3D view
ax = gca;
ax.Projection = 'orthographic';
hold on
%plot(points(:,1),points(:,2),'k')
axis([-1 1 -1 1])

plot3(eePos_des_traj(:,1),eePos_des_traj(:,2),eePos_des_traj(:,3),'r--') %desired path
plot3(eePos_sim(:,1),eePos_sim(:,2),eePos_sim(:,3),'b') %actual path

%%
% Set up an object to display the robot trajectory at a fixed rate of 15 frames per second. 
% Show the robot in each configuration from the inverse kinematic solver. 
% Watch as the arm traces the circular trajectory shown.
count = length(t_sim);
framesPerSecond = 100; %15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
