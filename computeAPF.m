function F = computeAPF(q, goal_coord, obstacles)
    K_att = 0.5; % 引力系数
    K_rep = 100;  % 斥力系数
    radius_rep = 150; % 斥力影响半径
    
    % 引力计算
    F_att = K_att * (goal_coord - q);
    
    % 斥力计算
    F_rep = [0, 0, 0];
    for i = 1:size(obstacles, 1)
        obs_center = obstacles(i, 1:3);
        obs_size = obstacles(i, 4:6);
        dist = norm(q - obs_center);
        
        % 检查是否在斥力影响范围内
        if dist < radius_rep
            dir_rep = (q - obs_center) / (dist + eps);
            rep_magnitude = K_rep * (1/dist - 1/radius_rep) * (1/dist^2);
            F_rep = F_rep + rep_magnitude * dir_rep;
        end
    end
    
    F = F_att + F_rep;
end