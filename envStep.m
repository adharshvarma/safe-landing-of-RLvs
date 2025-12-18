function [NextObs, Reward, IsDone, LoggedSignals] = envStep(Action, LoggedSignals)

    g = 9.81;
    rho = 1.225;
    m = 25600;
    S = 5.5;
    dt = 0.1;

    % --- SAFETY: clamp theta ---
    if any(~isfinite(Action))
        Action = 0;
    end
    theta = max(deg2rad(-20), min(deg2rad(20), Action));

    X = LoggedSignals.State;

    % Integrate dynamics
    dX = dynamics_RL(0, X, theta, m, S, rho, g);
    Xnext = X + dX * dt;

    % --- SAFETY: state check ---
    if any(~isfinite(Xnext))
        Xnext = X;
    end

    x  = Xnext(1);
    z  = Xnext(2);
    vx = Xnext(3);
    vz = Xnext(4);

    xf = 300;

    % Base reward (always finite)
    Reward = -abs(x - xf) - 0.01*abs(vx);

    % Termination
    IsDone = false;
    if z >= 0
        IsDone = true;

        Reward = Reward ...
            - 5*abs(vz) ...
            - 0.1*abs(vx) ...
            - 0.5*abs(x - xf);
    end

    % --- SAFETY: reward must be finite ---
    if ~isfinite(Reward)
        Reward = -1e6;
    end

    LoggedSignals.State = Xnext;
    NextObs = Xnext;
end
