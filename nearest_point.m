%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to find the nearest neighboring point.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function point = nearest_point(start, obstacle)
    options = optimoptions('quadprog','Display','off');
    len = length(start);
    point = quadprog(2*eye(len),-2*start,obstacle.A,obstacle.b,[],[],[],[],[],options);
end

