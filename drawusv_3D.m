function drawusv_3D_improved(x0, y0, z0, yaw, scale, Facecolor, FaceAlpha, flag)
% drawusv_3D_improved - 在三维平面图上绘制改进后的AUV图标
%
% 输入参数:
%   x0, y0, z0 - AUV 的中心点坐标
%   yaw - 偏航角（度）
%   scale - 缩放因子
%   Facecolor - AUV 图标的颜色
%   FaceAlpha - AUV 图标的透明度

% 参数配置
LengthSize = 2 * scale;    % AUV的长度
WidthSize = 1 * scale;     % AUV的宽度
HeightSize = 0.5 * scale;  % AUV的高度
ArrowLength = 1.5 * scale; % 箭头长度
ArrowWidth = 0.2 * scale;  % 箭头宽度

% 偏航角转换为弧度
psi = deg2rad(yaw);

% 构建长方体顶点（以中心点为基准）
vertices = [ 
    -LengthSize/2, -WidthSize/2, -HeightSize/2;
    LengthSize/2, -WidthSize/2, -HeightSize/2;
    LengthSize/2, WidthSize/2, -HeightSize/2;
    -LengthSize/2, WidthSize/2, -HeightSize/2;
    -LengthSize/2, -WidthSize/2, HeightSize/2;
    LengthSize/2, -WidthSize/2, HeightSize/2;
    LengthSize/2, WidthSize/2, HeightSize/2;
    -LengthSize/2, WidthSize/2, HeightSize/2;
];

% 长方体面的连接关系
faces = [
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8;
    1 2 3 4;
    5 6 7 8;
];

% 构建旋转矩阵
R = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi)  0;
     0        0         1];

% 旋转并平移长方体顶点
rotated_vertices = (R * vertices')';
rotated_vertices(:,1) = rotated_vertices(:,1) + x0;
rotated_vertices(:,2) = rotated_vertices(:,2) + y0;
rotated_vertices(:,3) = rotated_vertices(:,3) + z0;

% 绘制长方体
patch('Vertices', rotated_vertices, 'Faces', faces, ...
      'FaceColor', Facecolor, 'FaceAlpha', FaceAlpha, 'EdgeColor', 'k');

% 绘制方向箭头
arrow_base = [x0 + LengthSize/2 * cos(psi), y0 + LengthSize/2 * sin(psi), z0];
arrow_tip = [arrow_base(1) + ArrowLength * cos(psi), ...
             arrow_base(2) + ArrowLength * sin(psi), z0];

% 箭头的左右两翼
arrow_left = [arrow_tip(1) - ArrowWidth * sin(psi), ...
              arrow_tip(2) + ArrowWidth * cos(psi), z0];
arrow_right = [arrow_tip(1) + ArrowWidth * sin(psi), ...
               arrow_tip(2) - ArrowWidth * cos(psi), z0];

% 绘制箭头主体（直线）
plot3([arrow_base(1), arrow_tip(1)], ...
      [arrow_base(2), arrow_tip(2)], ...
      [arrow_base(3), arrow_tip(3)], 'b', 'LineWidth', 2);

% 绘制箭头头部
fill3([arrow_tip(1), arrow_left(1), arrow_right(1)], ...
      [arrow_tip(2), arrow_left(2), arrow_right(2)], ...
      [arrow_tip(3), arrow_left(3), arrow_right(3)], ...
      'r', 'FaceAlpha', 1);

% 添加中心点
scatter3(x0, y0, z0, 50, 'k', 'filled'); % 黑点表示中心点

hold on;
end
