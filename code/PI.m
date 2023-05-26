function PI(Kp_s,Ki_s,Kd_s,t,gp,g,H)

%{
<program>  Copyright (C) <2023>  <Huaqian Chin>
This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
This is free software, and you are welcome to redistribute it
under certain conditions; type `show c' for details.
%}

num = 10;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;
% 创建n序列 
noise = zeros(size(t));  % 创建n序列 
% 创建Td序列
Td = zeros(size(t));  % 创建Td序列

%% Kp不动Ki动
figure('Position', [10 10 900 900])
Kp = Kp_s;
for i = 1:5
    Ki = i*0.15;
    
    % PI function
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

    % 计算超调量
    overshoot = max(0, output - 10);  % 假设最终稳定值为10
    % 计算超调量变化比例
    overshoot_ratio = overshoot ./ 10;  % 假设最终稳定值为10
    % 找到最大超调量变化比例及其对应的时间
    [max_overshoot_ratio, idx] = max(overshoot_ratio);
    max_overshoot_ratio_time = t(idx);

    % 计算调节时间
    final_value = 10;  % 假设最终稳定值为10
    settling_index = find(output >= 0.98 * final_value & output <= final_value, 1, 'first');
    settling_time = t(settling_index) - 10;  

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    hold on;
    % 添加超调量标记
    text(max_overshoot_ratio_time, output(idx)-0.5, sprintf('Overshoot: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','top', 'HorizontalAlignment','left');
    % 添加调节时间标记
    line([settling_time + 10, settling_time + 10], ylim, 'Color', 'red', 'LineStyle', '--');
    text(settling_time + 10, output(settling_index)-0.5, sprintf('t_s = %.2f', settling_time), 'VerticalAlignment','top', 'HorizontalAlignment','right');
    hold off;

    title(['Response for PI Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki)])
    xlabel('Time (s)')
    ylabel('Response')
end

%% Ki不动Kp动
figure('Position', [10 10 900 900])
Ki = Ki_s;
for i = 1:5
    Kp = i*0.8;
    
    % PI function
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

    % 计算超调量
    overshoot = max(0, output - 10);  % 假设最终稳定值为10
    % 计算超调量变化比例
    overshoot_ratio = overshoot ./ 10;  % 假设最终稳定值为10
    % 找到最大超调量变化比例及其对应的时间
    [max_overshoot_ratio, idx] = max(overshoot_ratio);
    max_overshoot_ratio_time = t(idx);

    % 计算调节时间
    final_value = 10;  % 假设最终稳定值为10
    settling_index = find(output >= 0.98 * final_value & output <= final_value, 1, 'first');
    settling_time = t(settling_index) - 10;  

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    hold on;
    % 添加超调量标记
    text(max_overshoot_ratio_time, output(idx)-0.5, sprintf('Overshoot: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','top', 'HorizontalAlignment','left');
    % 添加调节时间标记
    line([settling_time + 10, settling_time + 10], ylim, 'Color', 'red', 'LineStyle', '--');
    text(settling_time + 10, output(settling_index)-0.5, sprintf('t_s = %.2f', settling_time), 'VerticalAlignment','top', 'HorizontalAlignment','right');
    hold off;

    title(['Response for PI Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki)])
    xlabel('Time (s)')
    ylabel('Response')
end


end