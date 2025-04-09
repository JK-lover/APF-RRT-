function [q_near, idx] = findNearest(nodes, target)
    all_coords = reshape([nodes.coord], 3, [])';
    dists = vecnorm(all_coords - target, 2, 2);
    [~, idx] = min(dists);
    q_near = nodes(idx);
end
