%% 2D Planar RLV Descent With P-Control (x0 = 0)
clear; clc; close all;

%% CONSTANTS
g = 9.81;
rho = 1.225;

% Vehicle geometry (Falcon 9-like)
m = 25600;
S = 5.5;

%% CONTROLLER PARAMETERS
Kp = 0.001;          % (let for now) tune later
theta_max = deg2rad(20);
theta_min = deg2rad(-20);

%% TARGET
xf = 300;             % let target landing 

%% INITIAL CONDITIONS
x0  = 0;              % start at 0
z0  = -2000;          % altitude (negative means downward)
vx0 = 0;
vz0 = 0;
X0  = [x0; z0; vx0; vz0];

%% TIME + EVENT (stop at ground)
opts = odeset('Events', @(t,X) ground_event(t,X));

tspan = [0 100];

%% ODE45
[t,X] = ode45(@(t,X) dynamics_PID(t,X,m,S,rho,g,Kp,theta_min,theta_max,xf), ...
               tspan, X0, opts);

%% Extract results
x  = X(:,1);
z  = X(:,2);
vx = X(:,3);
vz = X(:,4);

%% PLOTS 
figure;
plot(x,z,'LineWidth',2);
set(gca,'YDir','reverse');
xlabel('Horizontal position x (m)');
ylabel('Vertical position z (m)');
title('Trajectory with P-controller');
grid on;

figure;
plot(t,vx,'LineWidth',2); hold on;
plot(t,vz,'LineWidth',2);
legend('v_x','v_z');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs Time');
grid on;

figure;
subplot(2,1,1);
plot(t,x,'LineWidth',2);
xlabel('Time (s)'); ylabel('x (m)');
title('Horizontal Position vs Time');
grid on;

subplot(2,1,2);
plot(t,z,'LineWidth',2);
xlabel('Time (s)'); ylabel('z (m)');
title('Vertical Position vs Time');
grid on;

%% DYNAMICS + P-CONTROLLER
function dX = dynamics_PID(t,X,m,S,rho,g,Kp,theta_min,theta_max,xf)

    x  = X(1);
    z  = X(2);
    vx = X(3);
    vz = X(4);

    %% P-CONTROLLER
    e = xf - x;             % horizontal error
    theta_cmd = Kp * e;     % proportional output

    % Saturation limit
    theta = min(max(theta_cmd, theta_min), theta_max);

    %% VELOCITY
    V = sqrt(vx^2 + vz^2);

    %% Aerodynamics
    alpha = atan2(vx, vz) - theta;

    CL = 4*alpha;
    CD = 0.01 + 0.1*(alpha^2);

    q = 0.5 * rho * V^2;
    L = q * S * CL;
    D = q * S * CD;

    %% Equations of Motion
    dvx = -(D/m)*sin(theta+alpha) - (L/m)*cos(theta+alpha);
    dvz = g - (D/m)*cos(theta+alpha) - (L/m)*sin(theta+alpha);

    dX = [vx; vz; dvx; dvz];
end

%% EVENT: stops integration at z = 0
function [value, isterminal, direction] = ground_event(t,X)
    z = X(2);
    value = z;          % trigger at z = 0
    isterminal = 1;     % stop solver
    direction = 1;      % negative → zero
end
