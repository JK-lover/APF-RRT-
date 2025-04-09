%% 碰撞检测优化支持多个障碍物
function collision = checkCollision(p1, p2, obs)
    collision = false;
    for i = 1:size(obs,1)
        if lineIntersectsAABB(p1, p2, obs(i,:))
            collision = true;
            return;
        end
    end
end

% 判断线段与轴对齐包围盒（AABB）是否相交
function result = lineIntersectsAABB(p1, p2, box)
    t_min = 0;
    t_max = 1;
    for axis = 1:3
        if p1(axis) ~= p2(axis)
            t1 = (box(axis) - p1(axis)) / (p2(axis) - p1(axis));
            t2 = (box(axis) + box(axis+3) - p1(axis)) / (p2(axis) - p1(axis));
            t_min = max(t_min, min(t1, t2));
            t_max = min(t_max, max(t1, t2));
        elseif p1(axis) < box(axis) || p1(axis) > box(axis) + box(axis+3)
            result = false;
            return;
        end
    end
    result = (t_min <= t_max) && (t_max > 0) && (t_min < 1);
end