function [q_nearest, indices] = findNeighbors(nodes, center, radius)
    all_coords = reshape([nodes.coord], 3, [])';
    dists = vecnorm(all_coords - center, 2, 2);
    indices = find(dists <= radius);
    q_nearest = nodes(indices);
end
