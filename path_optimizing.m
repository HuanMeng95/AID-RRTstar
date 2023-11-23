function [optimized_path,optimized_v,optimized_w] = path_optimizing(path,scene,obstacle,start,goal,kappa)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this function utilizes convex polyhedral set and sliding window 
%                   techniques to optimize and obtain a trajectory.
% Input: 
%   path: feasible initial path
%   scene: The blank scene in the map
%   obstacle: The obstacles in the map
%   start: Start point information, including position and orientation
%   goal: goal point information, including position and orientation
%   kappa: The turning radius of AGV.
% Output: 
%   optimized_path: The desired performance trajectory.

% @file: path_optimizing.m
% @author: Huan Meng
% @update: 2023.11.23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.deltat = 0.1;
param.Lambda_e = 30;
param.Lambda_u = 1;
param.Lambda_s = 15;

param.max_vel = 0.8;  
param.body_size = 1;
n=20;  %sliding window length

body = Polyhedron([param.body_size param.body_size;param.body_size -param.body_size;-param.body_size ...
                                                 param.body_size;-param.body_size -param.body_size]);  %X_{sensor}
opt=sdpsettings('solver','fmincon','verbose',2);
[N,~] = size( path ); %The number of points in the path.

group_floor = floor( (N-1) / n); % Grouping the points in the path.
group_mod = mod(N-1,n);


obstacle_num= length(obstacle); %The number of obstacles.

x = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));
y = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));
theta = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));
v = sdpvar(repmat(1,1,n),repmat(1,1,n));
w = sdpvar(repmat(1,1,n),repmat(1,1,n));

xr = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));
yr = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));
thetar = sdpvar(repmat(1,1,n+1),repmat(1,1,n+1));


constraints = [];
objective = 0;  %total cost
objective1 = 0; objective2 = 0; objective3 = 0;

%Formulating an optimization problem.
for i = 1:n+1
    objective1 = objective1 + param.Lambda_e *  ( (x{i}-xr{i})^2 + (y{i}-yr{i})^2 +  (1-cos(thetar{i} - theta{i})) );%state cost
    if i <= n
        objective2 = objective2 + param.Lambda_u * ( (v{i})^2 + (w{i})^2 ); %input cost
    end
    if i <= n-1
        objective3 = objective3 + param.Lambda_s * ( ( x{i}+x{i+2}-2*x{i+1} )^2 + ( y{i}+y{i+2}-2*y{i+1} )^2 ); %smooth cost
    end
end
objective = objective1 + objective2 + objective3;

% kenimatics constraints and input constraints
for i = 1:n
    % kenimatics constraints
    constraints = [constraints,x{i+1}==param.deltat*(cos(theta{i})*v{i})+x{i}];
    constraints = [constraints,y{i+1}==param.deltat*(sin(theta{i})*v{i})+y{i}];
    constraints = [constraints,theta{i+1}==param.deltat*(w{i})+ theta{i}];
    % input constraints
    constraints = [constraints, (v{i})/param.max_vel + abs(w{i})/(param.max_vel*kappa) <= 1];
    constraints = [constraints, abs(w{i}/v{i})  <= kappa];
    constraints = [constraints, -v{i} <= -param.max_vel/2];
end
optimized_path = start;
optimized_v = [];
optimized_w = [];


%sliding window optimization
for i = 1: group_floor
    i
    total_constraints = [constraints, x{1} == optimized_path(end,1), y{1} == optimized_path(end,2),theta{1} == optimized_path(end,3) ...
        xr{1} == path((i-1)*n+1,1), yr{1} == path((i-1)*n+1,2), thetar{1} == path((i-1)*n+1,3) ];
    for m = 2:n+1
        temp_body = [cos(path( (i-1)*n+m,3)), -sin(path( (i-1)*n+m,3));sin(path( (i-1)*n+m,3)),cos(path( (i-1)*n+m,3)) ]*body +  path((i-1)*n+m,1:2)';
        intersec_poly(m) = intersect(scene,temp_body);
        temp_poly = [];
        for j = 1:obstacle_num
            temp_poly =[temp_poly,intersect(intersec_poly(m),obstacle(j))];
        end
        temp_intersect_poly =  PolyUnion(temp_poly);
        for j = 1:temp_intersect_poly.Num
            inner{m}(j,:) = nearest_point( path( (i-1)*n+m,1:2),temp_intersect_poly.Set(j));
            poly(m) = Polyhedron('A',-(path( (i-1)*n+m ,1:2) - inner{m}(j,:)) ,'b',-(path( (i-1)*n+m ,1:2)- inner{m}(j,:))*inner{m}(j,:)');
            intersec_poly(m) = intersect(intersec_poly(m) ,poly(m) );
        end
        %convex polyhedral set
        intersec_poly(m) = safe_region(intersec_poly(m));
        total_constraints = [ total_constraints, intersec_poly(m).A*[x{m};y{m}] <= intersec_poly(m).b,...
            xr{m} == path((i-1)*n+m,1), yr{m} == path((i-1)*n+m,2), thetar{m} == path((i-1)*n+m,3)  ];
        if i==group_floor && group_mod ==0
            total_constraints = [ total_constraints, x{n+1} == goal(1), y{n+1} == goal(2), cos(theta{n+1}) == cos(goal(3)),  sin(theta{n+1}) == sin(goal(3))];
            
        end
    end
    optimize(total_constraints,objective,opt);
    for k=2:length(x)
        optimized_path = [optimized_path;[double(x{k}),double(y{k}),double(theta{k})]];
        optimized_v = [optimized_v;double(v{k-1})];
        optimized_w = [optimized_w;double(w{k-1})];
    end
    
end
if  group_mod > 0
    total_constraints = [constraints, x{1} == optimized_path(end,1), y{1} == optimized_path(end,2),theta{1} == optimized_path(end,3)
        xr{1} == path(group_floor*n+1,1), yr{1} == path(group_floor*n+1,2), thetar{1} == path(group_floor*n+1,3) ];
    if group_floor >0
        num = group_mod+1;
    else
        num = group_mod;
    end
    for m = 2:num
        intersec_poly(m) = scene;
        for j = 1:obstacle_num
            inner{m}(j,:) = nearest_point( path( (group_floor)*n+m,1:2),obstacle(j));
            poly(m) = Polyhedron('A',-(path(group_floor*n+m ,1:2) - inner{m}(j,:)) ,'b',-(path(group_floor*n+m ,1:2)- inner{m}(j,:))*inner{m}(j,:)');
            %safe polyhedron
            intersec_poly(m) = intersect(intersec_poly(m) ,poly(m) ) ;
        end
        intersec_poly(m) = safe_region(intersec_poly(m));
        total_constraints = [ total_constraints, intersec_poly(m).A*[x{m};y{m}] <= intersec_poly(m).b,...
            xr{m} == path(group_floor*n+m,1), yr{m} == path(group_floor*n+m,2), thetar{m} == path(group_floor*n+m,3)];
        
        
        
    end
    total_constraints = [ total_constraints, x{num} == goal(1), y{num} == goal(2), cos(theta{num}) == cos(goal(3)), sin(theta{num}) == sin(goal(3))];
    optimize(total_constraints,objective,opt);
    for k=2:num
        optimized_path = [optimized_path;[double(x{k}),double(y{k}),double(theta{k})]];
        optimized_v = [optimized_v;double(v{k-1})];
        optimized_w = [optimized_w;double(w{k-1})];
    end
    
end

% end

