clear all
close all
clc

%system variables
M = 0.2; m = 0.012; b = 0;
l = 0.31; I = m*(2*l)^2/12; g = 9.81;

%linearized state matrices
denom = (l^2*M + I)*m + I*M;
A = [0 1 0 0 
     0 -(l^2*b*m+I*b)/denom -l^2*g*m^2/denom 0
     0 0 0 1
     0 b*l*m/denom (l*g*m^2 + m*l*M*g)/denom 0];
B = [0; (m*l^2 + I)/denom; 0; (-m*l)/denom];
C = [1 0 0 0
     0 0 0 0
     0 0 1 0
     0 0 0 0];
D = 0;

%weight matrices for LQR
Q = [1500 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 10];
R = 1;

%Full State Feedback Gain matrix found by LQR
K = lqr(A, B, Q, R)
%K = [-10.0000  -10.0  -59.5263  -10]

%check poles of closed loop system
eig(A - B*K)
sys = ss(A-B*K,B,C,D);
%impulse(sys);

%solving nonlinear system response
tspan = 0:0.001:20;
y0 = [0; 0.4; 0; 0];

options = odeset('OutputFcn', @(t, x_v, flag) output_force(t, x_v, flag, K));
[t, y] = ode45(@(t, x_v) nl_odesys(t, x_v, M, m, b, l, I, g, K), tspan, y0, options);

%nonlinear response plot
plot(t, y)
legend("x", "xdot", "thetea", "thetadot")

%realtime nonlinear simulation
 figure(2)
 sim_plot = plot(nan, nan, "square", nan, nan, "-", [-1 1], [0, 0], "-");
 sim_plot(2).LineWidth = 2;
 sim_plot(3).Color = "black"
 axis([-0.5 0.5 -0.2 0.8]);
 
 xc1 = y(1, 1);
 theta1 = y(1, 3);

 %plot initial position
 set(sim_plot(1), "XData", xc1, "YData", 0);
 set(sim_plot(2), "XData", [xc1, xc1 + 2*l*sin(theta1)], "YData", [0, 2*l*cos(theta1)]);
 pause(5);
 for i = 1:50:5000
     xc = y(i, 1);
     theta = y(i, 3);
     set(sim_plot(1), "XData", xc, "YData", 0);
     set(sim_plot(2), "XData", [xc, xc + 2*l*sin(theta)], "YData", [0, 2*l*cos(theta)]);
     pause(0.05);
end
figure(3)
plot(t, force)
disp("sim finished");


% nonlinear dynamics of pendulum for simulation
function x_dotv = nl_odesys(t, x_v, M, m, b, l, I, g, K)
    
    stheta = sin(x_v(3));
    ctheta = cos(x_v(3));
    denom2 = (l^2*m^2+l^2*M*m)*(stheta)^2 + l^2*M*m*(ctheta)^2 + I*m + I*M;
    f = -K*x_v;

    if abs(f) > 4
        f = 4 * abs(f)/f;
    end

    %f = 0;

    x_dotv(1, 1) = x_v(2);
    x_dotv(2, 1) = (stheta*(l^3*m^2*(ctheta)^2*x_v(4)^2 + I*l*m*x_v(4)^2 - l^2*g*m^2*ctheta) + l^3*m^2*(stheta)^3*x_v(4)^2 + (f - b*x_v(2)) * (l^2*m*(stheta)^2 + l^2*m*(ctheta)^2 + I) )/denom2;
    x_dotv(3, 1) = x_v(4);
    x_dotv(4, 1) = -(stheta*(l^2*m^2*ctheta*x_v(4)^2 - l*g*m^2 - l*M*g*m) + l*m*(f-b*x_v(2))*ctheta)/denom2;
end

function status = output_force(t,x_v,flag, K)
    persistent force
    switch flag
        case 'init'
            force = [0];
        case []
            f = -K*x_v;
            force = [force; transpose(f)];
        case 'done'
            assignin('base','force',force);
    end
    status = 0;
end