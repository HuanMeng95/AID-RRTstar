%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to maintain clearance between the
%                                trajectory and obstacles.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function B = safe_region(P)
    gamma = 1.25;
    m = P.chebyCenter;
    P1 =  P - m.x;
    B = Polyhedron('A',P1.A,'b',P1.b/gamma) + (m.x);
end

