function Out = AuvMathModel(In)
% In = [u; v; w; p; q; r(rad/s); phi(rad); theta(rad); tao(N,N,N,Nm,Nm,Nm)]
% Out = [dotu; dotv; dotw; dotp; dotq; dotr(rad/s/s)]

% 该模型来自《6-DoF Modelling and Control of a Remotely Operated Vehicle》

% 提取输入参数
u = In(1);        % 前进方向速度（m/s）
v = In(2);        % 横向速度（m/s）
w = In(3);        % 垂直速度（m/s）
p = In(4);        % 横滚角速度（rad/s）
q = In(5);        % 纵摇角速度（rad/s）
r = In(6);        % 偏航角速度（rad/s）
phi = In(7);      % 横滚角（rad）
theta = In(8);    % 纵摇角（rad）
tao = In(9:14);   % 控制输入（推力和力矩）
V = In(1:6);      % 速度向量

% 全局变量，包含AUV的物理参数和洋流速度分量
global auv V_cx V_cy V_cz;

% 计算相对速度，考虑洋流的影响
u_rel = u - V_cx; % 相对前进速度
v_rel = v - V_cy; % 相对横向速度
w_rel = w - V_cz; % 相对垂直速度

% 刚体科氏力矩阵 C_RB
C_RB = [0 0 0 0 auv.m*w 0;
        0 0 0 -auv.m*w 0 0;
        0 0 0 auv.m*v -auv.m*u 0;
        0 auv.m*w -auv.m*v 0 auv.Izz*r -auv.Iyy*q;
        -auv.m*w 0 -auv.m*u -auv.Izz*r 0 auv.Ixx*p;
        auv.m*v -auv.m*u 0 auv.Iyy*q -auv.Ixx*p 0];

% 附加质量科氏力矩阵 C_A
C_A = [0 0 0 0 auv.Zdotw*w 0;
       0 0 0 -auv.Zdotw*w 0 -auv.Xdotu*u;
       0 0 0 -auv.Ydotv*v auv.Xdotu*u 0;
       0 -auv.Zdotw*w auv.Ydotv*v 0 auv.Ndotr*r auv.Mdotq*q;
       auv.Zdotw*w 0 -auv.Xdotu*u auv.Ndotr*r 0 -auv.Kdotp*p;
       -auv.Ydotv*v auv.Xdotu*u 0 -auv.Mdotq*q auv.Kdotp*p 0];

% 阻力矩阵 D，使用相对速度计算
D = -diag([auv.Xu + auv.Xuu * abs(u_rel), ...
           auv.Yv + auv.Yvv * abs(v_rel), ...
           auv.Zw + auv.Zww * abs(w_rel), ...
           auv.Kp + auv.Kpp * abs(p), ...
           auv.Mq + auv.Mqq * abs(q), ...
           auv.Nr + auv.Nrr * abs(r)]);

% 重力和浮力矩阵 g
g = [(auv.W - auv.B) * sin(theta);
    -(auv.W - auv.B) * cos(theta) * sin(phi);
    -(auv.W - auv.B) * cos(theta) * cos(phi);
    auv.zg * auv.W * cos(theta) * sin(phi);
    auv.zg * auv.W * sin(theta);
    0];

% 合力和合力矩计算
Fsum = tao - C_RB * V - C_A * V - D * V - g;

% 计算各自由度上的加速度
Out = auv.Mni * Fsum;
end
