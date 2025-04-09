clearvars
close all

%% 环境参数
x_max = 1000;
y_max = 1000;
z_max = 1000;
obstacles = [
    300 400 400 200 200 200;
    500 200 300 150 150 250]; % [x y z width height depth]

%% rrt*算法参数
EPS = 50;            % 步长
numNodes = 12000;    % 最大节点
rewire_radius = 120; % 重布线半径
goal_bias = 0.15;    % 目标偏向采样概率

%% 节点结构体初始化
q_start = struct('coord', [0 0 0], 'cost', 0, 'parent', 0);
q_goal = struct('coord', [x_max y_max z_max], 'cost', 0, 'parent', []);
nodes = q_start; % 初始化节点数组

%% 可视化设置
figure('Name','3D RRT*','Position',[100 100 1200 800])
axis equal
xlim([0 x_max]), ylim([0 y_max]), zlim([0 z_max])
view(135,30)
grid on
hold on

% 绘制障碍物
for i = 1:size(obstacles,1)
    plotcube(obstacles(i,4:6), obstacles(i,1:3), 0.5, [0.3 0.3 0.3]);
end

% 标记起终点
plot3(q_start.coord(1), q_start.coord(2), q_start.coord(3),...
      'pentagram','MarkerSize',20,'Color','b','LineWidth',3);
plot3(q_goal.coord(1), q_goal.coord(2), q_goal.coord(3),...
      'hexagram','MarkerSize',20,'Color','m','LineWidth',3);

%% 主算法循环
path_found = false;
for i = 1:numNodes
    if rand < goal_bias
        q_rand = q_goal.coord;
    else
        % 使用APF偏向采样（建议比例0.3，可调参数）
        if rand < 0.3 % APF偏向采样概率
            F = computeAPF(q_near.coord, q_goal.coord, obstacles);
            if norm(F) > eps
                q_rand = q_near.coord + EPS * (F / norm(F));
                % 约束在环境范围内
                q_rand = max([0,0,0], min([x_max,y_max,z_max], q_rand));
            else
                q_rand = [randi(x_max), randi(y_max), randi(z_max)];
            end
        else
            q_rand = [randi(x_max), randi(y_max), randi(z_max)];
        end
    end
    
    % 寻找最近节点（向量化加速）
    all_coords = reshape([nodes.coord], 3, [])';
    [~, idx] = min(vecnorm(all_coords - q_rand, 2, 2));
    q_near = nodes(idx);
    
    % 生成新节点
    direction = q_rand - q_near.coord;
    if norm(direction) < eps
        continue;  % 避免方向为零的情况
    end
    step_size = min(norm(direction), EPS);
    q_new.coord = q_near.coord + (direction/norm(direction)) * step_size;
    q_new.cost = q_near.cost + step_size;
    q_new.parent = idx;

    % 应用APF调整
    alpha = 15; % 调整步长
    F = computeAPF(q_new.coord, q_goal.coord, obstacles);
    if norm(F) > eps
        adjusted_coord = q_new.coord + alpha * (F / norm(F));
        % 限制调整后坐标在有效区域内
        adjusted_coord = max([0,0,0], min([x_max,y_max,z_max], adjusted_coord));
        % 检查调整后的路径是否无碰撞
        if ~checkCollision(q_near.coord, adjusted_coord, obstacles)
            q_new.coord = adjusted_coord;
        end
    end
    
    % 碰撞检测
    if checkCollision(q_near.coord, q_new.coord, obstacles)
        continue;
    end
    
    % 绘制探索路径
    line([q_near.coord(1), q_new.coord(1)],...
         [q_near.coord(2), q_new.coord(2)],...
         [q_near.coord(3), q_new.coord(3)],...
         'Color',[0.7 0.7 0.7], 'LineWidth',0.5);
    
    % 邻域重布线优化
    [q_nearest, indices] = findNeighbors(nodes, q_new.coord, rewire_radius);
    [q_new, updated] = rewire(q_new, q_nearest, indices, obstacles);
    
    % 添加节点到列表
    nodes(end+1) = q_new; % 修正后结构体一致
    
    % 目标检测
    if norm(q_new.coord - q_goal.coord) < EPS
        q_goal.parent = length(nodes);
        nodes(end+1) = q_goal;
        path_found = true;
        fprintf('路径找到! 迭代次数: %d\n', i);
        break;
    end
    
    % 动态显示更新
    if mod(i,200) == 0
        drawnow limitrate
        title(['探索进度: ', num2str(100*i/numNodes,'%.1f'), '%']);
    end
end

%% 路径回溯
if path_found
    % 逆向追踪路径
    path = [];
    q_end = q_goal;
    total_cost = 0;
    
    while q_end.parent ~= 0
        path = [q_end.coord; path];
        start = q_end.parent;
        line([q_end.coord(1), nodes(start).coord(1)],...
             [q_end.coord(2), nodes(start).coord(2)],...
             [q_end.coord(3), nodes(start).coord(3)],...
             'Color','r', 'LineWidth',3);
        total_cost = total_cost + norm(q_end.coord - nodes(start).coord);
        q_end = nodes(start);
    end
    
    % 添加起点
    path = [q_start.coord; path];
    
    % 显示路径信息
    fprintf('路径总长度: %.2f units\n', total_cost);
    title(sprintf('3D RRT* 路径规划 | 长度: %.2f | 节点数: %d',...
          total_cost, length(nodes)));
else
    error('未找到路径! 请尝试调整参数');
end


