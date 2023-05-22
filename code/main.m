clc;clear;close all;

%% 初始默认参数
Kp_s = 2.8;
Ki_s = 0.15;
Kd_s = 3;
p = 2;
t = 0:0.05:100;  % 定义时间区间

%% Pump Function
n = [1 0];  % s
gp = tf(1, n);

%% Patient Function
n = [1, 2*p, p*p];
g = tf(1, n);

%% H(s)
H = tf(1, 1);


%% --------------case 1.1.1 :  PD  ------------------
%
%       R = 10 (t>10);  N(s) = 0;  Td(s) = 0;
%
%  在这个模块中，将PD接入了系统的Gc(s)的位置
%  实现了对两个参数 (Kp & Kd) 进行了两组分析
%   1. 保持Kp不变，调整Kd的值————以1为步长，在[1,5]的范围内生成5张系统时域图
%   2. 保持Kd不变，调整Kp的值————以2为步长，在[2,10]的范围内生成5张系统时域图
% --------------------------------------------------
PD(Kp_s,Ki_s,Kd_s,t,gp,g,H);



%% --------------case 1.1.2 :  PI  ------------------
%
%       R = 10 (t>10);  N(s) = 0;  Td(s) = 0;
%
%  在这个模块中，将PI接入了系统的Gc(s)的位置
%  实现了对两个参数 (Kp & Ki) 进行了两组分析
%   1. 保持Kp不变，调整Ki的值————以0.15为步长，在[0.15,0.75]的范围内生成5张系统时域图
%   2. 保持Ki不变，调整Kp的值————以0.8为步长，在[0.8,4]的范围内生成5张系统时域图
% --------------------------------------------------
% PI(Kp_s,Ki_s,Kd_s,t,gp,g,H)




%% --------------case 1.2 :  PID  ------------------
%
%       R = 10 (t>10);  N(s) = 0;  Td(s) = 0;
%
%  在这个模块中，将PID接入了系统的Gc(s)的位置
%  实现了对三个参数 (Kp & Ki) 进行了三组分析
%   1. 保持Kp & Ki不变，调整Kd的值————以0.8为步长，在[0.8,4]的范围内生成5张系统时域图
%   2. 保持Kp & Kd不变，调整Ki的值————以0.15为步长，在[0.15,0.75]的范围内生成5张系统时域图
%   2. 保持Ki & Kd不变，调整Kp的值————以0.6为步长，在[0.6,3]的范围内生成5张系统时域图
% --------------------------------------------------
% PID(Kp_s,Ki_s,Kd_s,t,gp,g,H)


%% --------------case 2 :  PID with Noise  ------------------
%
%           R = 0;  N(s) = 0;  Td(s) = 50/s ;
%
%  在这个模块中，将PID接入了系统的Gc(s)的位置
%  实现了对三个参数 (Kp & Ki) 进行了三组分析
%   1. 保持Kp & Ki不变，调整Kd的值————以0.8为步长，在[0.8,4]的范围内生成5张系统时域图
%   2. 保持Kp & Kd不变，调整Ki的值————以0.15为步长，在[0.15,0.75]的范围内生成5张系统时域图
%   2. 保持Ki & Kd不变，调整Kp的值————以0.6为步长，在[0.6,3]的范围内生成5张系统时域图
% ------------------------------------------------------------
PID_with_noise(Kp_s,Ki_s,Kd_s,t,gp,g,H)
