clc; clear all; close all;

% Intended final result:
% The robot moves through the map on a predetermined path searching for objects on the ground.
% If the robot finds something using its sensors, it will pick it up and plan a new path back to the starting point.

% The path is planned using the PRM algorithm and connecting the waypoints in a way that
% ensures a collision free path. For smooth operation we use the trajectory planner “Pure Pursuit” 
% to follow the path through the environment. When the robot reaches the end without finding any 
% objects on the ground, it will return home to its “home-base”. This part is implemented in the code,
% although the pick-up and return to home part is not. 

load exampleMaps.mat;
map = binaryOccupancyMap(simpleMap);

prmComplex = mobileRobotPRM(map, 150); % Increase the number of nodes to improve path quality. But increases computational cost.
prmComplex.ConnectionDistance = 10; % Set the maximum distance between nodes.

% Define the waypoints
waypoints = [2 2;   % Start
             8 8;   % mid1
             2 23;  % mid2
             10 23; % mid3
             12 16; % mid4
             20 20; % mid5
             15 9;  % mid6
             24 9;  % mid7
             15 3;  % mid8
             24 3;  % End
             2 2];  % Return to Start

% Generate the path through the waypoints
completePath = waypoints(1,:);
for i = 1:(size(waypoints,1)-1)
    startLocation = waypoints(i,:);
    endLocation = waypoints(i+1,:);
    interimPath = findpath(prmComplex, startLocation, endLocation);
    if isempty(interimPath)
        error('No path found between %d and %d', i, i+1);
    end
    % Append the path, excluding the first node to avoid duplicates
    completePath = [completePath; interimPath(2:end,:)];
end

% Define the robot with differential drive kinematics
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Define the controller with the complete path
controller = controllerPurePursuit;
controller.Waypoints = completePath;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;

% Define simulation parameters
sampleTime = 0.1; % Sample time [s]
vizRate = rateControl(1/sampleTime); % Visualization rate

% Start the simulation loop
robotCurrentPose = [completePath(1,:) 0]';
goalRadius = 0.1;
isReturning = false; % Indicate that the robot has reached the end and is returning home,
                     % to ensure that the simulation does not stop
                     % immediately, since the start and end location is at
                     % the same point

% Initialize the figure
figure

% Plot the environment
show(prmComplex);
hold on;

% Plot the complete path
plot(completePath(:,1), completePath(:,2),"k--d");

while true
    
    % Compute the controller outputs, the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel * sampleTime; 
    
    % Check if the robot has reached the final waypoint before returning
    if ~isReturning && norm(robotCurrentPose(1:2)' - waypoints(end-1,:)) <= goalRadius
        isReturning = true; % The robot is now returning to start
    end
    
    % Re-compute the distance to the start
    distanceToStart = norm(robotCurrentPose(1:2)' - waypoints(1,:));
    
    % Plot the robot's current location
    if isReturning
        % Plot return path in green
        plot(robotCurrentPose(1), robotCurrentPose(2), 'go', 'MarkerSize', 2);
    else
        % Plot path to end in red
        plot(robotCurrentPose(1), robotCurrentPose(2), 'ro', 'MarkerSize', 2);
    end

    % Update visualization
    xlim([0 27]);
    ylim([0 26]);
    
    waitfor(vizRate);
    
    % Check if the robot has returned to start
    if isReturning && distanceToStart <= goalRadius
        break; % Stop the simulation
    end
end

% Hold off the plot
hold off;
