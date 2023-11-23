%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to define obstacle.
% Input:   
%     vertex: Vertices of the obstacle
%     offset: Offset of the obstacle
% Output: 
%    obstacle: Polyhedron information of the obstacle.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function obstacle = Obstacle(vertex,offset)
    obstacle = Polyhedron(vertex) + offset;
end

