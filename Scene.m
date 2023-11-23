%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to define scene.
% Input:   
%     vertex: Vertices of the scene
%     offset: Offset of the scene
% Output: 
%    scene: Polyhedron information of the scene.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function scene = Scene(vertex,offset)
    scene = Polyhedron(vertex) + offset;
end

