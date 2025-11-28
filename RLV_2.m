%% 2D Planar RLV Aerodynamic Descent Simulation
clear; clc;close all;

%% CONSTANTS
g = 9.81;                 % gravity (m/s^2)
rho = 1.225;              % air density (kg/m^3)

% Falcon 9 approx (first stage)
m = 25600;                % kg - dry mass + residuals
S = 5.5;                  % m^2 - reference body area

%% CONSTANT PITCH ANGLE (radians)
theta = deg2rad(10);      % choose constant theta

%% ODE INITIAL CONDITIONS
x0  = 0;                  % initial horizontal position
z0  = -2000;              % 2 km altitude
vx0 = 0;                 % m/s horizontal speed
vz0 = 0;               % m/s downward
X0  = [x0; z0; vx0; vz0];

%% SIMULATION TIME
tspan = linspace(0,20.2,100);

%% RUN SIMULATION
[t, X] = ode45(@(t,X) dynamics( X, m, S, rho, g, theta), tspan, X0);

%% EXTRACT STATES
x  = X(:,1);
z  = X(:,2);
vx = X(:,3);
vz = X(:,4);

%% PLOTTING -----------------------------------------------------

% Trajectory
figure; 
plot(x, z, 'LineWidth', 2);
set(gca, 'YDir','reverse');   % because z is positive downward
xlabel('Horizontal (m)');
ylabel('Vertical Downwards (m)');
title('2D Planar Rocket Descent');
grid on;

% Velocity time histories
figure;
plot(t, vx, t, vz, 'LineWidth', 2);
legend('v_x','v_z');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

% NEW: position vs time plots (your request)
figure;
subplot(2,1,1);
plot(t, x, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('x Position (m)');
title('Horizontal Position vs Time');
grid on;

subplot(2,1,2);
plot(t, z, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('z Position (m)');
title('Vertical Position vs Time');
grid on;

%% ------------------ DYNAMICS FUNCTION ------------------ %%
function dX = dynamics( X, m, S,rho, g, theta)

    % States
    x  = X(1); %#ok<NASGU>
    z  = X(2); %#ok<NASGU>
    vx = X(3);
    vz = X(4);

    % Magnitude of velocity

    V = sqrt(vx^2 + vz^2);

    % Angle of attack
    alpha = atan2(vx, vz) - theta;

    % Aerodynamic coefficients
    CL = 4 * alpha;
    CD = 0.01 + 0.1 * (alpha^2);

    % Aerodynamic forces
    q = 0.5 * rho * V^2;
    L = q * S * CL;
    D = q * S * CD;

    % Governing equations from your notebook
    dvx = -(D/m)*sin(theta+alpha) - (L/m)*cos(theta+alpha);
    dvz = g - (D/m)*cos(theta+alpha) - (L/m)*sin(theta+alpha);
    
    % Assemble derivative vector
    dX = [ vx;      % x dot
           vz;      % z dot
           dvx;     % vx dot
           dvz ];   % vz dot
end
