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

% --------------------------------------
%     gc = 1;
%     gc = gc_pi;
%     gc = gc_pd;
%     gc = gc_pid;
% --------------------------------------


% --------------case 1.1.1 :  PD  ------------------
% N(s) = 0;  Td(s) = 0;
gc = gc_pd;

% 传递函数
fai_r = (gc*gp*g)/(1+H*gc*gp*g);
fai_Td = (g)/(1+H*gc*gp*g);
fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);

% r = ones(size(t)) * 10;% 创建r序列
num = 50;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;

n = zeros(size(t));  % 创建n序列 
Td = zeros(size(t));  % 创建Td序列

output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
output_n = lsim(fai_n, n, t);   % 对应fai_n的输出
output = output_r + output_Td + output_n; % 得到总输出 

figure;
plot(t, output);  % 画出时域图像
title('Response for PD Controller with N=0 and Td=0')
xlabel('Time (s)')
ylabel('Response')



% --------------case 1.1.2 :  PI  ------------------
% N(s) = 0;  Td(s) = 0;
gc = gc_pi;

% 传递函数
fai_r = (gc*gp*g)/(1+H*gc*gp*g);
fai_Td = (g)/(1+H*gc*gp*g);
fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);

% % r = ones(size(t)) * 10;  % 创建r序列
num = 50;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;

n = zeros(size(t));  % 创建n序列 
Td = zeros(size(t));  % 创建Td序列

output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
output_n = lsim(fai_n, n, t);   % 对应fai_n的输出
output = output_r + output_Td + output_n; % 得到总输出 

figure;
plot(t, output);  % 画出时域图像
title('Response for PI Controller with N=0 and Td=0')
xlabel('Time (s)')
ylabel('Response')


% --------------case 1.2 :  PID  ------------------
% N(s) = 0;  Td(s) = 0;
gc = gc_pid;

% 传递函数
fai_r = (gc*gp*g)/(1+H*gc*gp*g);
fai_Td = (g)/(1+H*gc*gp*g);
fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);

% r = ones(size(t)) * 0;  % 创建r序列
num = 50;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;

n = zeros(size(t));  % 创建n序列 
Td = zeros(size(t));  % 创建Td序列

output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
output_n = lsim(fai_n, n, t);   % 对应fai_n的输出
output = output_r + output_Td + output_n; % 得到总输出 

figure;
plot(t, output);  % 画出时域图像
title('Response for PID Controller with N=0 and Td=0')
xlabel('Time (s)')
ylabel('Response')


% --------------case 2 :  PID with Noise  ------------------
% N(s) = 0;  Td(s) = 50/s;
gc = gc_pid;

% 传递函数
fai_r = (gc*gp*g)/(1+H*gc*gp*g);
fai_Td = (g)/(1+H*gc*gp*g);
fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);

r = ones(size(t)) * 0;  % 创建r序列
n = zeros(size(t));  % 创建n序列 
% 创建Td序列
num = 50;
Td = zeros(size(t));  % 初始化为全零
Td(t > 10 & t <= 11) = Td(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
Td(t > 11) = num; 

output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
output_n = lsim(fai_n, n, t);   % 对应fai_n的输出
output = output_r + output_Td + output_n; % 得到总输出 

figure;
plot(t, output);  % 画出时域图像
title('Response for PID Controller with N=0 and Td=50/s')
xlabel('Time (s)')
ylabel('Response')










% % 创建r序列
% % r = ones(size(t)) * 10;
% r = zeros(size(t));
% 
% % 创建n序列
% n = zeros(size(t));  % 初始化为全零
% % n(t > 30 & t <= 31) = n(t > 30 & t <= 31) + (max(0, min((t(t > 30 & t <= 31) - 30), 1)) * 10);
% % n(t > 31) = 10;
% 
% % 创建Td序列
% num = 50;
% Td = zeros(size(t));  % 初始化为全零
% Td(t > 10 & t <= 11) = Td(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
% Td(t > 11) = num;
% 
% 
% output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
% output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
% output_n = lsim(fai_n, n, t);   % 对应fai_n的输出
% 
% % 得到总输出
% output = output_r + output_Td + output_n;  
% 
% plot(t, output);  % 画出时域图像



