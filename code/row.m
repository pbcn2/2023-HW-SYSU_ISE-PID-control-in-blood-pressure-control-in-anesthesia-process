function row(Kp_s,Ki_s,Kd_s,t,gp,g,H)
num = 10;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;
% 创建n序列 
noise = zeros(size(t));  % 创建n序列 
% 创建Td序列
Td = zeros(size(t));  % 创建Td序列

%% 
figure(1);

gc = 1;
fai_r = (gc*gp*g)/(1+H*gc*gp*g);
fai_Td = (g)/(1+H*gc*gp*g);
fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
output = output_r + output_Td + output_n; % 得到总输出

% 计算调节时间
final_value = 10;  % 假设最终稳定值为10
settling_index = find(output >= 0.98 * final_value & output <= final_value, 1, 'first');
settling_time = t(settling_index) - 10;  % 调节时间从t=10开始计算

plot(t, output);  % 画出时域图像

% 添加输入开始的标记
hold on;
line([10, 10], ylim, 'Color', 'green', 'LineStyle', '--');
text(10, min(output), '', 'VerticalAlignment','top', 'HorizontalAlignment','left');

% 添加调节时间标记
line([settling_time + 10, settling_time + 10], ylim, 'Color', 'red', 'LineStyle', '--');
text(settling_time + 10, min(output), sprintf('t_s = %.2f', settling_time), 'VerticalAlignment','top', 'HorizontalAlignment','right');
hold off;

title('Response for row Controller')
xlabel('Time (s)')
ylabel('Response')


%%
figure(1);

Td= zeros(size(t));  % 初始化为全零
Td(t > 50 & t <= 51) = Td(t > 50 & t <= 51) + (max(0, min((t(t > 50 & t <= 51) - 10), 1)) * 50);
Td(t > 51) = 50;

gc = 1;
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

plot(t, output);  % 画出时域图像
hold on;
plot(t, overshoot_ratio, 'r--');  % 画出超调量变化比例曲线
text(max_overshoot_ratio_time, max_overshoot_ratio, sprintf('Max overshoot ratio: %.2f%%', max_overshoot_ratio*100), 'VerticalAlignment','bottom', 'HorizontalAlignment','right');
hold off;

title('Response and Overshoot Ratio for row Controller---Td_input')
xlabel('Time (s)')
ylabel('Response / Overshoot Ratio')
legend('Response', 'Overshoot Ratio')

end

