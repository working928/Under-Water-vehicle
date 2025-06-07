clear all
close all
clc

tic;  % 开始计时
%% 自定义的内容
global Vc theta_c phi_c V_cx V_cy V_cz;

surf_Vc = 2; % 水面洋流速度
theta_c = pi/4; % 洋流在xy平面的方向（从X轴往Y轴旋转）
phi_c = 0.995; % 洋流在z方向的角度（从XOY平面抬起来的角度）

user_target_P = [-120 -120 -80]; % 目标位置
user_init_p = [0 0 -40]; % 起始位置
target_Yaw = 0; % 目标艏向（定义了目标在达到目标位置时的目标艏向）
yaw = rand * 165; % 初始艏向为0-360的随机值

%% 用户定义内容结束，默认海域深度为100m
% 洋流速度方向的计算
Vc = surf_Vc * 0.69;
V_cx = Vc * cos(theta_c) * sin(phi_c); % 洋流的x分量
V_cy = Vc * sin(theta_c) * sin(phi_c); % 洋流的y分量
V_cz = Vc * cos(phi_c); % 洋流的z分量

target_vector = user_target_P - user_init_p % 目标向量

% 洋流方向向量
current_vector = [V_cx, V_cy, V_cz]

% 计算夹角
cos_theta = dot(target_vector, current_vector) / (norm(target_vector) * norm(current_vector));
theta = acos(cos_theta); % 计算夹角

% 打印夹角的大小
disp(['夹角为: ', num2str(rad2deg(theta)), '°']); 

% 判断方向
if theta < pi/4
    disp('目标方向接近洋流方向');
    V_cx1 = V_cx;
    V_cy1 = V_cy;
    V_cz1 = V_cz;
    
elseif theta > 3 * pi/4
    disp('目标方向与洋流方向相反');

    % 计算目标方向单位向量
    target_dir = target_vector / norm(target_vector);

    % 洋流在目标方向上的分量（投影）
    Vc_proj_mag = dot(current_vector, target_dir);  % 投影的大小（可能为负）
    Vc_proj_vec = Vc_proj_mag * target_dir;         % 投影向量

    % 减弱逆流投影分量，最多减去 0.5 m/s
    if Vc_proj_mag < 0
        % 是逆向，进行减弱
        reduce_mag = min(abs(Vc_proj_mag), (Vc * 0.3));  % 不能减过头
        Vc_proj_reduced = (Vc_proj_mag + reduce_mag) * target_dir;
    else
        % 若不是逆流，不做处理
        Vc_proj_reduced = Vc_proj_vec;
    end

    % 正交方向上的分量
    Vc_orth = current_vector - Vc_proj_vec;

    % 合成新的洋流矢量
    Vc_total = Vc_orth + Vc_proj_reduced;

    % 拆解成 x, y, z 分量
    V_cx1 = Vc_total(1);
    V_cy1 = Vc_total(2);
    V_cz1 = Vc_total(3);

else
    disp('目标方向与洋流方向大致垂直');
    % 计算目标方向单位向量
    target_dir = target_vector / norm(target_vector);
    
    % 求垂直方向向量（假设在XY平面上）
    % 使用一个90°旋转向量即可
    orth_dir = [-target_dir(2), target_dir(1), 0];  % 在XY平面上的垂直方向
    
    % 添加扰动：设扰动大小为 Vc2
    Vc2 = Vc * 0.5; % 尾流增强量
    perturbation = Vc2 * orth_dir;
    
    % 叠加尾流
    V_cx1 = V_cx + perturbation(1);
    V_cy1 = V_cy + perturbation(2);
    V_cz1 = V_cz + perturbation(3);  % Z方向不变

end

%% 进行两段仿真，第一段，从起点到尾流影响区
% 计算第一段仿真的目标点（尾流影响区入口，离最终目标1米）
direction = user_target_P - user_init_p;
unit_dir = direction / norm(direction);
first_segment_end = user_target_P - unit_dir * 2; % 退1米作为第一段目标

% 设置第一段仿真的起点和终点
init_p = user_init_p;         % 第一段起点就是用户定义的起点
target_P = first_segment_end % 第一段目标点是尾流入口

disp(norm(user_target_P - target_P));  % 应该是 1（或非常接近）

figure;
set(gca,'linewidth',1.5, FontName', 'Times New Roman',)

%% 将目标位姿设置到simulink中
 for i = 1:2
    set_param('ROVSim_dp', 'SimulationMode', 'accelerator');
    set_param('ROVSim_dp/Cmd Yaw 0-360 [Deg]', 'Value', num2str(target_Yaw));
    set_param('ROVSim_dp/Cmd Position X Y Z [m]', 'Value', sprintf('[%d %d %d]', ...
              target_P(1), target_P(2), target_P(3)));
    
    % 循环仿真
    set_param('ROVSim_dp/Kinematics', 'x0', num2str(init_p(1), '%.2f'), ...
              'y0', num2str(init_p(2), '%.2f'), ...
              'z0', num2str(init_p(3), '%.2f'), ...
              'yaw0', num2str(yaw, '%.2f')); % 设置初始位姿
    sim('ROVSim_dp');


    %% 获取最后一个仿真坐标点
    X = logsout{25}.Values.Data;
    Y = logsout{26}.Values.Data;
    Z = logsout{27}.Values.Data;
    Yaw = logsout{3}.Values.Data;  % 先取出数据

    % 获取最后一个点（仿真结束时的位置）
    state1_x = X(end);
    state1_y = Y(end);
    state1_z = Z(end);
    
    % 拼成一个向量
    state1_position = [state1_x, state1_y, state1_z];


    % 这里更新艏向为第一次仿真的终点艏向

    yaw = Yaw(end);                % 再取最后一个艏向值 

    % 显示结果
    disp(sprintf('第%d段仿真终点坐标： [%s]', i, num2str(state1_position)));

    %% 在图中标记出洋流的方向
    hold on; grid on; axis equal;
    xlabel('X [m]','FontSize',20, 'FontName', 'Palatino Linotype');
    ylabel('Y [m]','FontSize',20, 'FontName', 'Palatino Linotype');
    zlabel('Z [m]','FontSize',20, 'FontName', 'Palatino Linotype');
    
    % 绘制仿真轨迹
    X = logsout{25}.Values.Data;
    Y = logsout{26}.Values.Data;
    Z = logsout{27}.Values.Data;
    Yaw = logsout{3}.Values.Data;

    % 随机生成轨迹颜色
    color = [rand rand rand];
    % 绘制三维轨迹
    plot3(Y, X, Z, 'color', color, 'LineWidth', 3);
    
    % 在每一段起点绘制AUV图标
    drawusv_3D(Y(1), X(1), Z(1), Yaw(1), 1, color, 0.3);
    
    % 设置坐标轴刻度字体大小为20
    set(gca, 'FontSize', 20);
    view(3);
    
    Vc_show = sqrt(V_cx^2 + V_cy^2 + V_cz^2);
    % 绘制洋流方向的箭头
    quiver3(Y(1), X(1), Z(1), V_cx, V_cy, V_cz, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    text(Y(1), X(1), Z(1), sprintf('Current: %.1f m/s', Vc_show), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 20, 'FontName', 'Palatino Linotype');

    % 在终点绘制AUV图标
    drawusv_3D(Y(end), X(end), Z(end), Yaw(end), 1, color, 0.3);

    V_cx = V_cx1;
    V_cy = V_cy1;
    V_cz = V_cz1;
    current_vector = [V_cx, V_cy, V_cz];
    init_p = state1_position;
    target_P = user_target_P;
 end

% 标记目标位置
drawusv_3D(user_target_P(2), user_target_P(1), user_target_P(3), target_Yaw, 1, [0 0 0], 0);

% 绘制尾流影响区（球体）
[spX, spY, spZ] = sphere(30);           % 生成球面坐标（30x30 网格）
r = 2;                                  % 半径为1米
surf(spY * r + user_target_P(2), ...
     spX * r + user_target_P(1), ...
     spZ * r + user_target_P(3), ...
     'FaceAlpha', 0.3, ...              % 半透明
     'EdgeColor', 'none', ...
     'FaceColor', [0.2, 0.6, 1]);       % 淡蓝色球体

% title('ROV 3D Trajectory with Ocean Current', 'FontName', 'Palatino Linotype');

hold off;

elapsedTime = toc;  % 结束计时，返回耗时（单位：秒）
fprintf('运行时间: %.4f 秒\n', elapsedTime);