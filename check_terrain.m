function [x_new, y_new, z_new, yaw_new] = check_terrain(P_current, step_size, terrain_data, X, Y, target_P)
    x = P_current(1);
    y = P_current(2);
    z = P_current(3);
    yaw = P_current(4);

    % 计算前方位置的坐标
    x_next = x + step_size * cosd(yaw);
    y_next = y + step_size * sind(yaw);
    
    % 使用插值方法获取前方位置的地形高度
    z_terrain = interp2(X, Y, terrain_data, x_next, y_next);

    % 如果前方地形高度高于当前深度，调整潜水器深度
    if z_terrain > z
        z_new = z_terrain + 1; % 上浮一定距离
    else
        z_new = z;
    end

    % 更新新位置和新的偏航角
    x_new = x_next;
    y_new = y_next;
    
    % 更新偏航角，如果朝向目标位置发生改变，则更新yaw
    delta_x = target_P(1) - x_new;
    delta_y = target_P(2) - y_new;
    yaw_new = atan2d(delta_y, delta_x);
end
