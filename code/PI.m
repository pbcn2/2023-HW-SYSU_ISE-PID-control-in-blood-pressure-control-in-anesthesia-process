function PI(Kp_s,Ki_s,Kd_s,t,gp,g,H)

num = 50;
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

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
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

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    title(['Response for PI Controller with Kp=',num2str(Kp) ,'  Ki=', num2str(Ki)])
    xlabel('Time (s)')
    ylabel('Response')
end


end