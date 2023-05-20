function y = pump_block(input_signal, t)

    % 系统分子多项式系数向量
    n = [1 0];

    % 构建传递函数对象
    gc = tf(1, n);

    % 时间向量
    t_ = 0:0.01:t;

    % 计算响应
    y = lsim(gc, input_signal, t_);

end