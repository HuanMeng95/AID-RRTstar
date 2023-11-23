%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%In this main function, the first step is to find the feasible initial path
%             for the AGVs. Subsequently, the obtained path is optimized to
%            generate a trajectory.
% @paper: AID-RRT* Based High-Quality Trajectory planner
%                  for Autonomous Guided Vehicles
% @author: Huan Meng
% @update: 2023.11.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all,clc;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Four different maps have been defined, with the
%            scene size and obstacles being defined within each map.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MAP = ["map1","map2","map3","map4"];
cmap = MAP(3);
switch cmap
    case "map1"
        scene = Scene([0 0;0 5;5 0;5 5],[0;0]);
        plot(scene,'alpha',0);
        hold on
        vertex1 = [0.5 2;0.5 3;2 2;2 3];
        offset1 = [0;0];
        obstacle(1) = Obstacle(vertex1,offset1);
        vertex2 = [0.5 2;0.5 3;2 2;2 3];
        offset2 = [2.5;0];
        obstacle(2) = Obstacle(vertex2,offset2);
        plot(obstacle,'Color','black');
        start = [1,1,pi/2];
        goal = [4.5, 3.5, 0];
    case "map2"
        scene = Scene([0 0;0 10;10 0;10 10],[0;0]);
        plot(scene,'alpha',0);
        hold on
        vertex1 = [0 2;0 3;7 2;7 3];
        offset1 = [0;0];
        obstacle(1) = Obstacle(vertex1,offset1);
        vertex2 = [5 5;4 5;5 10;4 10];
        offset2 = [0;0];
        obstacle(2) = Obstacle(vertex2,offset2);
        vertex3 = [5 6;5 7;2 6;2 7];
        offset3 = [0;0];
        obstacle(3) = Obstacle(vertex3,offset3);
        plot(obstacle,'Color','black');
        start = [1,1,pi/4];
        goal = [3, 9, pi];
    case "map3"
        scene = Scene([0 0;0 15;15 0;15 15],[0;0]);
        plot(scene,'alpha',0);
        hold on
        vertex1 = [4 13;3 12;5 11;4 10];
        offset1 = [0;1];
        obstacle(1) = Obstacle(vertex1,offset1);
        vertex2 = [4 13;3 12;5 11;4 10];
        offset2 = [0;-4];
        obstacle(2) = Obstacle(vertex2,offset2);
        vertex3 = [4 13;3 12;5 11;4 10];
        offset3 = [0;-9];
        obstacle(3) = Obstacle(vertex3,offset3);
        vertex4 = [4 13;3 12;5 11;4 10];
        offset4 = [7;1];
        obstacle(4) = Obstacle(vertex4,offset4);
        vertex5 = [4 13;3 12;5 11;4 10];
        offset5 = [7;-4];
        obstacle(5) = Obstacle(vertex5,offset5);
        vertex6 = [4 13;3 12;5 11;4 10];
        offset6 = [7;-9];
        obstacle(6) = Obstacle(vertex6,offset6);
        vertex7 = [6 10;8 10;7 11;7 9];
        offset7 = [0.5;0];
        obstacle(7) = Obstacle(vertex7,offset7);
        vertex8 = [6 10;8 10;7 11;7 9];
        offset8 = [0.5;-5];
        obstacle(8) = Obstacle(vertex8,offset8);
        plot(obstacle,'Color','black');
        start = [1,14,-pi/2];
        goal = [14, 1, 0];
    otherwise
        scene = Scene([0 0;0 20;20 0;20 20],[0;0]);
        plot(scene,'alpha',0);
        hold on
        vertex1 = [4 18;5 18;4 12;5 12];
        offset1 = [0;-9];
        obstacle(1) = Obstacle(vertex1,offset1);
        vertex2 = [4 18;5 18;4 12;5 12];
        offset2 = [0;0];
        obstacle(2) = Obstacle(vertex2,offset2);
        vertex3 =  [4 18;5 18;4 12;5 12];
        offset3 = [4;-2];
        obstacle(3) = Obstacle(vertex3,offset3);
        vertex4 =  [4 18;5 18;4 12;5 12];
        offset4 = [4;-11];
        obstacle(4) = Obstacle(vertex4,offset4);
        vertex5 =  [12 18;18 18;12 17;18 17];
        offset5 = [0;0];
        obstacle(5) = Obstacle(vertex5,offset5);
        vertex6 =  [12 18;18 18;12 17;18 17];
        offset6 = [1;-5];
        obstacle(6) = Obstacle(vertex6,offset6);
        vertex7 =  [12 18;18 18;12 17;18 17];
        offset7 = [0;-10];
        obstacle(7) = Obstacle(vertex7,offset7);
        vertex8 =  [12 18;18 18;12 17;18 17];
        offset8 = [1;-15];
        obstacle(8) = Obstacle(vertex8,offset8);
        vertex9 = [4 18;5 18;4 12;5 12];
        offset9 = [-3;-5];
        obstacle(9) = Obstacle(vertex9,offset9);
        plot(obstacle,'Color','black');
        start = [18,4,pi/2];
        goal = [2, 18, -pi/2];
end
xlabel('X (m)');
ylabel('Y (m)');
grid off
set(gca,'Box','on')
scatter(start(1),start(2),80,'o','filled','MarkerFaceColor',[0.8,0.4,0])
scatter(goal(1),goal(2),120,'p','filled','MarkerFaceColor',[0.3,0.6,0.2])
planner_name = 'Adaptive_dubins_informed_rrt_star';
planner = str2func(planner_name);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%obtaining the feasible initial path has been obtained
%by employing the adaptive_dubins_informed_rrt_star search algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[path, flag, cost, expand_node, expand_edge, kappa] = planner(scene,obstacle, start, goal);
% optimized
if flag == false
    disp('Error, no path found!');
    return;
end
dubins_path.path = path;
dubins_path.smooth = sum(sqrt (sum (diff( diff(path(:,1:2)) ).^2,2)));
dubins_Path.distance = cost;

h1 = plot(dubins_path.path(:,1),dubins_path.path(:,2), 'b', 'Linewidth', 2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%obtaining a trajectory by optimizing the raw path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[optimized_path,~,~] = path_optimizing(path,scene,obstacle,start,goal,kappa);

Trajectory.optimized_path = optimized_path;
Trajectory.cost = sum( sqrt(sum(diff(optimized_path(:,1:2)).^2,2)) );
Trajectory.smooth = sum(sqrt (sum (diff( diff(optimized_path(:,1:2)) ).^2,2)));
h2 = plot(optimized_path(:,1),optimized_path(:,2),'-m','linewidth',3);
legend([h1,h2],'Raw path','Optimized trajectory')
hold off
