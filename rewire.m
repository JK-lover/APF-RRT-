function [q_new, updated] = rewire(q_new, q_nearest, indices, obstacles)
    updated = false;
    min_cost = q_new.cost;
    
    for k = 1:length(q_nearest)
        if ~checkCollision(q_nearest(k).coord, q_new.coord, obstacles)
            tentative_cost = q_nearest(k).cost + norm(q_new.coord - q_nearest(k).coord);
            
            if tentative_cost < min_cost
                min_cost = tentative_cost;
                q_new.parent = indices(k);
                updated = true;
            end
        end
    end
    q_new.cost = min_cost;
end