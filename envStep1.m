function [NextObs, Reward, IsDone, LoggedSignals] = envStep1(Action, LoggedSignals)

    g = 9.81;
    rho = 1.225;
    m = 25600;
    S = 5.5;
    dt = 0.1;

    % Discrete action mapping
    thetaSet = deg2rad([-10 0 10]);
    theta = thetaSet(Action);   % Action = 1,2,3

    X = LoggedSignals.State;

    % Integrate dynamics
    dX = dynamics_RL1(0, X, theta, m, S, rho, g);
    Xnext = X + dX * dt;

    x  = Xnext(1);
    z  = Xnext(2);
    vz = Xnext(4);

    xf = 300;

    % Reward (simple & stable)
    Reward = -abs(x - xf);

    % Termination at ground
    IsDone = false;
    if z >= 0
        IsDone = true;
        Reward = Reward - 2*abs(vz);   % soft landing penalty
    end

    LoggedSignals.State = Xnext;
    NextObs = Xnext;
end
