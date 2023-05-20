function y = PID_block(Kp, Ki, Kd, input_signal, t)

    % 系统分子多项式系数向量
    n = [1 0];

    % 计算系统分母多项式系数向量
    d = [Kd, Kp, Ki];

    % 构建传递函数对象
    gc = tf(d, n);

    % 时间向量
    t_ = 0:0.01:t;

    % 计算响应
    y = lsim(gc, input_signal, t_);

end
