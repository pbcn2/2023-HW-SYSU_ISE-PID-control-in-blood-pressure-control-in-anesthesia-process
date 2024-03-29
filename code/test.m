clc;clear;close all;

% 参数
Kp = 1;
Ki = 0.5;
Kd = 0.5;
p = 2;
t = 0:0.05:100;  % 定义时间区间

% PID function
n = [Kd, Kp, Ki];  %分子多项式系数向量
d = [1 0];  %分母多项式向量
gc_pid = tf(n, d); % 构建传递函数对象

% PD function
n = [Kd, Kp, 0];  %分子多项式系数向量
d = [1 0];  %分母多项式向量
gc_pd = tf(n, d);% 构建传递函数对象

% PI function
n = [0, Kp, Ki];  %分子多项式系数向量
d = [1 0];  %分母多项式向量
gc_pi = tf(n, d);% 构建传递函数对象

% Pump Function
n = [1 0];  % s
gp = tf(1, n);

% Patient Function
n = [1, 2*p, p*p];
g = tf(1, n);

% H(s)
H = tf(1, 1);

% 创建r序列
num = 50;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;

% 创建n序列 
noise = zeros(size(t));  % 创建n序列 
% 创建Td序列
Td = zeros(size(t));  % 创建Td序列

% 固定Kp，调整Ki
figure('Position', [10 10 900 900])
for i = 1:5
    Ki = i*0.1;
    n = [0, Kp, Ki];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pi = tf(n, d);% 构建传递函数对象
    gc = gc_pi;
    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    
    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    title(['Response for PID Controller with Ki=', num2str(Ki)])
    xlabel('Time (s)')
    ylabel('Response')
end
