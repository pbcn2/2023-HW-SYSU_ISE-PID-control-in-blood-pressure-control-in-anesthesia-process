function PID_with_noise(Kp_s,Ki_s,Kd_s,t,gp,g,H)

%{
<program>  Copyright (C) <2023>  <Huaqian Chin>
This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
This is free software, and you are welcome to redistribute it
under certain conditions; type `show c' for details.
%}

% N(s) = 0;  Td(s) = 50/s;
r = 10 * ones(size(t));  % 初始化为全零
% 创建n序列 
noise = zeros(size(t));  
% 创建Td序列
num = 50;
Td = zeros(size(t));  % 初始化为全零
Td(t > 50 & t <= 51) = Td(t > 50 & t <= 51) + (max(0, min((t(t > 50 & t <= 51) - 50), 1)) * num);
Td(t > 51) = num; 

%% Kp&Ki不动Kd动
figure('Position', [10 10 900 900])
Kp = Kp_s;
Ki = Ki_s;
for i = 1:5
    Kd = i*0.8;
    
    % PID function
    n = [Kd, Kp, Ki];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pid = tf(n, d); % 构建传递函数对象
    gc = gc_pid;

    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    % 计算超调量
    overshoot = max(0, output - 10);  % 假设最终稳定值为10
    % 计算超调量变化比例
    overshoot_ratio = overshoot ./ 10;  % 假设最终稳定值为10
    % 找到最大超调量变化比例及其对应的时间
    [max_overshoot_ratio, idx] = max(overshoot_ratio);
    max_overshoot_ratio_time = t(idx);

    subplot(5,1,i)
%     plot(t, output);  % 画出时域图像
%     hold on;
%     % 添加超调量标记
%     text(max_overshoot_ratio_time, output(idx)-0.5, sprintf('Overshoot: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','top', 'HorizontalAlignment','left');
    plot(t, output);  % 画出时域图像
    hold on;
    plot(t, overshoot_ratio, 'r--');  % 画出超调量变化比例曲线
    text(max_overshoot_ratio_time, max_overshoot_ratio, sprintf('Max overshoot ratio: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','bottom', 'HorizontalAlignment','right');
    hold off;

    title(['Response for PID Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki),'  Kd=', num2str(Kd)])
    xlabel('Time (s)')
    ylabel('Response')
end

%% Kp&Kd不动Ki动
figure('Position', [10 10 900 900])
Kp = Kp_s;
Kd = Kd_s;
for i = 1:5
    Ki = i*0.15;
    
    % PID function
    n = [Kd, Kp, Ki];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pid = tf(n, d); % 构建传递函数对象
    gc = gc_pid;

    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    % 计算超调量
    overshoot = max(0, output - 10);  % 假设最终稳定值为10
    % 计算超调量变化比例
    overshoot_ratio = overshoot ./ 10;  % 假设最终稳定值为10
    % 找到最大超调量变化比例及其对应的时间
    [max_overshoot_ratio, idx] = max(overshoot_ratio);
    max_overshoot_ratio_time = t(idx);

    subplot(5,1,i)
%     plot(t, output);  % 画出时域图像
%     hold on;
%     % 添加超调量标记
%     text(max_overshoot_ratio_time, output(idx)-0.5, sprintf('Overshoot: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','top', 'HorizontalAlignment','left');
    plot(t, output);  % 画出时域图像
    hold on;
    plot(t, overshoot_ratio, 'r--');  % 画出超调量变化比例曲线
    text(max_overshoot_ratio_time, max_overshoot_ratio, sprintf('Max overshoot ratio: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','bottom', 'HorizontalAlignment','right');
    hold off;

    title(['Response for PID Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki),'  Kd=', num2str(Kd)])
    xlabel('Time (s)')
    ylabel('Response')
end

%% Ki&Kd不动Kp动
figure('Position', [10 10 900 900])
Ki = Ki_s;
Kd = Kd_s;
for i = 1:5
    Kp = i*0.6;
    
    % PID function
    n = [Kd, Kp, Ki];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pid = tf(n, d); % 构建传递函数对象
    gc = gc_pid;

    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    % 计算超调量
    overshoot = max(0, output - 10);  % 假设最终稳定值为10
    % 计算超调量变化比例
    overshoot_ratio = overshoot ./ 10;  % 假设最终稳定值为10
    % 找到最大超调量变化比例及其对应的时间
    [max_overshoot_ratio, idx] = max(overshoot_ratio);
    max_overshoot_ratio_time = t(idx);

    subplot(5,1,i)
%     plot(t, output);  % 画出时域图像
%     hold on;
%     % 添加超调量标记
%     text(max_overshoot_ratio_time, output(idx)-0.5, sprintf('Overshoot: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','top', 'HorizontalAlignment','left');
    plot(t, output);  % 画出时域图像
    hold on;
    plot(t, overshoot_ratio, 'r--');  % 画出超调量变化比例曲线
    text(max_overshoot_ratio_time, max_overshoot_ratio, sprintf('Max overshoot ratio: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','bottom', 'HorizontalAlignment','right');
    hold off;

    title(['Response for PID Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki),'  Kd=', num2str(Kd)])
    xlabel('Time (s)')
    ylabel('Response')
end

end