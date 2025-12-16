function dX = dynamics_RL1(t, X, theta, m, S, rho, g)

    x  = X(1);
    z  = X(2);
    vx = X(3);
    vz = X(4);

    V = max(1e-3, sqrt(vx^2 + vz^2));

    alpha = atan2(vx, vz) - theta;

    CL = 4 * alpha;
    CD = 0.01 + 0.1 * alpha^2;

    q = 0.5 * rho * V^2;
    L = q * S * CL;
    D = q * S * CD;

    dvx = -(D/m)*sin(theta+alpha) - (L/m)*cos(theta+alpha);
    dvz = g - (D/m)*cos(theta+alpha) - (L/m)*sin(theta+alpha);

    dX = [vx; vz; dvx; dvz];
end
