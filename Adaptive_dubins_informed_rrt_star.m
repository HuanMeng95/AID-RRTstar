function [path, flag_find, cost, expand_node, expand_edge, kappa] = Adaptive_dubins_informed_rrt_star(scene,obstacle,start, goal)
%%
% @origin file: informed_rrt.m
% @breif: Informed RRT* motion planning
% @paper: Optimal Sampling-based Path Planning Focused via Direct
%                   Sampling of an Admissible Ellipsoidal heuristic
% @author: Winter
% @update: 2023.2.2

% @file: Adaptive_dubins_informed_rrt_star.m
% @breif: Adaptive_dubins_informed_rrt_star
% @paper: AID-RRT* Based High-Quality Trajectory planner
%                  for Autonomous Guided Vehicles
% @author: Huan Meng
% @update: 2023.11.23

% Input:
%   scene: The blank scene in the map
%   obstacle: The obstacles in the map
%   start: Start point information, including position and orientation
%   goal: goal point information, including position and orientation
% Output:
%   path: The generated feasible initial path
%   flag_find: The flag for finding the path. If successful, it is set to true.
%   cost: The length of the initial path.
%   expand_node:The expansion status of Adaptive Dubins Informed RRT* nodes.
%   expand_edge:The connectivity status of Adaptive Dubins Informed RRT* nodes.
%   kappa: The turning radius of AGV.
%%
% optimal radius
param.r = 1;
% Maximum expansion distance one step
param.max_dist = 3;
% Maximum number of sample points
param.sample_num = 1000;
% heuristic sample
param.goal_sample_rate = 0.05;
%scene center
param.scene_center = sum(scene.V)/4;
% scene size
param.x_range = abs(scene.V(1,1)-param.scene_center(1))*2;
param.y_range = abs(scene.V(1,2)-param.scene_center(2))*2;
% big number
param.big_num = 10000;
% best planning cost
param.c_best = param.big_num;
%minimum turning radius and maximum curvature
param.turning_rad = 0.354;
kappa = 1/param.turning_rad;
%sampling resolution of path
param.resolution = 0.06;
%normlized
param.segments =param.resolution /  param.turning_rad;
% distance between start and goal
[ param.c_min,~] = dubins_searchn(start, goal, param.turning_rad);
% sample list

% Start point, cost, parent node, parent node index.
sample_list  = [start, 0, start ,1];

[~,sample_edge_list{1}]=dubins_searchn(start, start, param.turning_rad);
path_ = [];
path = [];
flag = false;
flag_find = false;
cost = 0;
expand_node = [];
expand_edge = [];
h = [];
% main loop
for i=1: param.sample_num
    [cost_, flag, sample_list, sample_edge_list,  path_, expand_node, expand_edge] = plan(sample_list, sample_edge_list, expand_node,  expand_edge, start, goal, scene, obstacle, param);
    pause(0.01);
    new_branch = [dubins_path(sample_edge_list{end},param.segments)';sample_list(end,1:3)]';
    plot(new_branch(1,:),new_branch(2,:),'r');
    plot(sample_list(end,1),sample_list(end,2),'*r');
    if flag && cost_ < param.c_best
        param.c_best = cost_;
        path = path_;
        cost = cost_;
        flag_find = flag_find || flag;
        delete(h);
        h=[];
        h = plot(path(:,1),path(:,2), 'b', 'Linewidth', 3);
    end
end
end

%%
function index = loc_list(node, list, range)
% @breif: locate the node in given list
num = size(list);
index = 0;
if ~num(1)
    return
else
    for i=1:num(1)
        if isequal(node(range), list(i, range))
            index = i;
            return;
        end
    end
end
end

function [cost, flag, node_list, edge_list, path, expand_node, expand_edge] = plan(node_list, edge_list,  expand_node,  expand_edge, start, goal, scene, obstacle, param)
cost = 0;
flag = false;
path = [];

% generate a random node in the scene
node_rand = generate_node(start, goal, param);

% visited
if loc_list(node_rand, node_list, [1, 3])
    return
end
% generate new node
[node_new,new_param_start, nearNode, success] = get_nearest(node_list, node_rand, scene, obstacle, param);

%
if success
    node_list = [node_list;node_new];
    temp_edge_list{1} = new_param_start;
    edge_list = [edge_list;temp_edge_list];
    [node_list,edge_list] = rewire(node_list, edge_list, nearNode, node_new,scene, obstacle, param);
    [node_num, ~] = size(node_list);
    [distance,param_start]=dubins_searchn(node_new(1:3), goal(1:3), param.turning_rad);
    %
    %plot(node_rand(1),node_rand(2),'.b');
    % goal found
    if distance <= param.max_dist && ~is_collision(node_new, goal, scene, obstacle, param)
        goal_ = [goal, node_new(4) + distance, node_new(1:3),node_num];
        if ~isequal(node_new(1:3), goal)
            node_list = [node_list;goal_];
            temp_edge_list{1} = param_start;
            edge_list = [edge_list;temp_edge_list];
        end
        cost = goal_(4);
        flag = true;
        path = extract_path(node_list, edge_list, start, param);
        if ~isequal(path(end, 1:3), goal)
            path = [path;goal];
        end
        expand_node = node_list;
        expand_edge = edge_list;
        node_list(end, :) = [];
        edge_list(end) = [];
        return
    end
end
end

function node = generate_node(start, goal, param)
%breif: Generate a random node to extend exploring tree.
% ellipse sample
if param.c_best < param.big_num
    while true
        % unit ball sample
        p = [0, 0, 1];
        while true
            x = -1 + 2 * rand();
            y = -1 + 2 * rand();
            theta = 2*pi*rand();
            if x * x + y * y < 1
                p(1) = x; p(2) = y;
                break
            end
        end
        % transform to ellipse
        p_star = transform(param.c_best / 2, param.c_min / 2, start, goal) * p';
        if param.scene_center(1) - param.x_range/2 <= p_star(1) &&  p_star(1) <=  param.scene_center(1) + param.x_range/2 && ...
                param.scene_center(2) - param.y_range/2 <= p_star(2) && p_star(2) <= param.scene_center(2) + param.y_range/2
            node = [p_star(1), p_star(2), theta];
            return;
        end
    end
    %     % random sample
else
    if rand() > param.goal_sample_rate
        x = param.x_range/2 * (2*rand()-1) +  param.scene_center(1);
        y = param.y_range/2 * (2*rand()-1) +  param.scene_center(2);
        theta = 2*pi*rand();
        node = [x, y, theta];
        return
    end
    
    node = goal;
    return
end
end

function [new_node,new_param_start, nearNode, flag] = get_nearest(node_list, node, scene, obstacle, param)
%@breif: Get the node from `node_list` that is nearest to `node`.
flag = false;
% find nearest neighbor
dist_vector = dist(node_list(:, 1:2), node(1:2)');
[~, index] = min(dist_vector);
node_near = node_list(index, :);

% regular and generate new node
%   distance = min(dist(node_near(1:2), node(1:2)'), param.max_dist);
distance = steer(node_near,node,param);

theta = angle(node_near, node);
new_node = [node_near(1) + distance * cos(theta), ...
    node_near(2) + distance * sin(theta), ...
    node(3),...
    0, ...
    node_near(1:3),...
    index];
[param.r,~] = dubins_searchn(node_near, node, param.turning_rad);
nearNode = [];
% obstacle check
[collision_flag, stage_cost,param_start,~] = is_collision(node_near, new_node , scene, obstacle,  param);
new_param_start = param_start;
if collision_flag
    return
end


nearNode = [nearNode;index];
new_node(4) = node_near(4) + stage_cost;
cost = new_node(4);

%  nearC && chooseParent
[node_num, ~] = size(node_list);
for i=1:node_num
    if i==index
        continue;
    end
    node_n = node_list(i, :);
    %  inside the optimization circle
    new_dist = dist(node_n(1:2), new_node(1:2)');
    if new_dist < param.r
        [collision_flag, stage_cost,param_start,~] = is_collision(node_n, new_node, scene, obstacle,  param);
        costTmp = node_n(4) + stage_cost;
        %  update new sample node's cost and choose parent
        if  ~collision_flag
            nearNode = [nearNode; i];
            if cost > costTmp
                cost = costTmp;
                new_node(4) = cost;
                new_node(5:7) = node_n(1:3);
                new_node(8) = i;
                new_param_start = param_start;
            end
        end
    end
end
flag = true;
end

function [flag,stage_cost,param_start,path] = is_collision(node_start, node_end, scene, obstacle, param)
%@breif: Judge collision when moving from node1 to node2.
%node1: start point; node2: ending point
node1 = node_start(1:3);
node2 = node_end(1:3);
flag = true;
[stage_cost,param_start]=dubins_searchn(node1, node2, param.turning_rad);
path = dubins_path(param_start,param.segments);
[~,path_num]=size(path);
obstacle_num = length(obstacle);
for i=1:path_num
    for j = 1:obstacle_num
        if sum(obstacle(j).A*[path(1,i);path(2,i)] <= obstacle(j).b) == length(obstacle(j).b)
            return;
        end
    end
    if sum(-scene.A* [path(1,i);path(2,i)] <= -scene.b) > 0
        return
    end
end
flag = false;
end

function path = extract_path(close_node,close_edge, start, param)
% @breif: Extract the path based on the CLOSED set.
path  = [];
closeNum = length(close_node(:, 1));
index = closeNum;
while 1
    path_node = dubins_path(close_edge{index},param.segments);
    path = [ path_node';path];
    if isequal(close_edge{index}.p_init, start(1:3))
        break;
    end
    index = close_node(index,8);
end
end

function theta = angle(node1, node2)
theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
end

function T = transform(a, c, start, goal)
% center
center_x = (start(1) + goal(1)) / 2;
center_y = (start(2) + goal(2)) / 2;

% rotation
theta =  -angle(start, goal);

% transform
b = sqrt(a * a - c * c);
T = [ a * cos(theta), b * sin(theta),  center_x; ...
    -a * sin(theta), b * cos(theta), center_y; ...
    0,                      0,            1];
end


function [node_list, edge_list]= rewire(node_list, edge_list, nearNode,node_new,scene, obstacle, param)
[node_num, ~] = size(nearNode);
[Tnum, ~] =  size(node_list);
for i = 1:node_num-1
    if node_new(8) ==  node_list(nearNode(i),8)
        continue;
    end
    node1 = node_list(nearNode(i),:);
    node2 = node_new;
    [collision_flag, stage_cost,param_start,path] = is_collision(node2, node1, scene, obstacle,  param);
    
    if collision_flag
        continue;
    end
    
    if node1(4) > node2(4) + stage_cost
        param_start_last = edge_list{nearNode(i)};
        path_last = dubins_path(param_start_last,param.segments);
        plot( path_last(1,:),path_last(2,:),'-w');
        node_list(nearNode(i),4) = node_new(4) + stage_cost;
        node_list(nearNode(i),5:7) = node2(1:3);
        node_list(nearNode(i),8) = Tnum;
        edge_list{nearNode(i)} = param_start;
        plot(path(1,:),path(2,:),'r');
        
    end
end
end

function distance = steer(node_start,node_end,param)
% @breif: Adaptive strategy for step size
dx = node_end(1) - node_start(1);
dy = node_end(2) - node_start(2);
theta = atan2( dy, dx );
eta = 0.5*(cos(node_start(3) - theta )  +  cos(node_end(3) - theta )  );
D = 4*param.turning_rad ;
L = param.max_dist;
distance = 0.5*( (L+D) + eta*(D-L) );
end









