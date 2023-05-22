function PD(Kp_s,Ki_s,Kd_s,t,gp,g,H)

% N(s) = 0;  Td(s) = 0;
num = 10;
r = zeros(size(t));  % 初始化为全零
r(t > 10 & t <= 11) = r(t > 10 & t <= 11) + (max(0, min((t(t > 10 & t <= 11) - 10), 1)) * num);
r(t > 11) = num;
% 创建n序列 
noise = zeros(size(t));  % 创建n序列 
% 创建Td序列
Td = zeros(size(t));  % 创建Td序列

%% Kp不动Kd动
figure('Position', [10 10 900 900])
Kp = Kp_s;
for i = 1:5
    Kd = i*1;

    % PD Block
    n = [Kd, Kp, 0];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pd = tf(n, d);% 构建传递函数对象
    gc = gc_pd;

    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    title(['Response for PD Controller with Kp=',num2str(Kp) ,'  Kd=', num2str(Kd)])
    xlabel('Time (s)')
    ylabel('Response')
end

%% Kd不动Kp动
figure('Position', [10 10 900 900])
Kd = Kd_s;
for i = 1:5
    Kp = i*2;
    
    % PD Block
    n = [Kd, Kp, 0];  %分子多项式系数向量
    d = [1 0];  %分母多项式向量
    gc_pd = tf(n, d);% 构建传递函数对象
    gc = gc_pd;

    fai_r = (gc*gp*g)/(1+H*gc*gp*g);
    fai_Td = (g)/(1+H*gc*gp*g);
    fai_n = (H*gc*gp*g)/(1+H*gc*gp*g);
    output_r = lsim(fai_r, r, t);  % 对应fai_r的输出
    output_Td = lsim(fai_Td, Td, t);  % 对应fai_Td的输出
    output_n = lsim(fai_n, noise, t);   % 对应fai_n的输出
    output = output_r + output_Td + output_n; % 得到总输出  

    subplot(5,1,i)
    plot(t, output);  % 画出时域图像
    title(['Response for PD Controller with Kp=',num2str(Kp) ,'  Kd=', num2str(Kd)])
    xlabel('Time (s)')
    ylabel('Response')
end

end
