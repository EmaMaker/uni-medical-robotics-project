clc
clear

%% preliminary computations and plots
global L1 L2 L3 L MAXT
global xExtended yExtended xCurled yCurled
global angle angleE angleC xM yM R

% index
% lengths
L1 = 0.0435;
L2 = 0.0333;
L3 = 0.0204;
MAXT = pi;

% middle
% L1 = 0.0468
% L2 = 0.0345
% L3 = 0.0228
% R=0.072

% ring
% L1 = 0.0469
% L2 = 0.0345
% L3 = 0.0228
% 
% R = 0.072

% pinky
% L1 = 0.0337
% L2 = 0.0252
% L3 = 0.0135
% 
% R = 0.052

% thumb
% L1 = 0.0461
% L2 = 0.0414
% L3 = 0.0241
% R = 0.08;
%v=f([deg2rad(40); deg2rad(50); pi/2]);
v = f(ones(3,1)*pi*0.5);
xCurled = v(1);
yCurled = v(2);

L=L1+L2+L3;


closure = 0:0.01:1;
xExtended=0;
yExtended=L1+L2+L3;

%xCurled=L1-L3
%yCurled=-L2

xM = 0.5*(xCurled+xExtended)
yM = 0.5*(yCurled+yExtended)

syms q1 q2 q3 real
global Jtask q tfin

q=[q1;q2;q3];
Jtask = simplify(jacobian(f(q), q));

% null space term to keep joint within limits
H = 0;
qMax = deg2rad(80);
qMin = 0;
qMiddle = (qMax+qMin)*0.5;
for i=1:3
    H = H + ((q(i)-qMiddle)/(qMax-qMin))^2;
    %H = H + (q(i)/pi)^2;
end
H = H/6;
dq0 = -gradient(H, q);


figure()
hold on


xlim([-L+L1, L+L1])
ylim([-L+L1, L+L1])

% extended
line([0, 0], [0, L1], 'LineStyle','--', 'LineWidth',1,'Color','red')
line([0, 0], [L1, L1+L2], 'LineStyle','--','LineWidth',1, 'Color','blue')
line([0, 0], [L1+L2, L1+L2+L3], 'LineStyle','--', 'LineWidth',1,'Color','green')

% curled
line([0, L1], [0, 0], 'LineStyle','--', 'LineWidth',1,'Color','red')
line([L1, L1], [0, -L2], 'LineStyle','--','LineWidth',1, 'Color','blue')
line([L1, L1-L3], [-L2, -L2], 'LineStyle','--', 'LineWidth',1,'Color','green')

% center
plot(0,0,'Marker','o','MarkerSize',10)
% extended positions
plot(xExtended,yExtended,'Marker','*','MarkerSize',10)
plot(xCurled, yCurled, 'Marker','*','MarkerSize',10)
plot(xM, yM, 'Marker', 'x', 'MarkerSize',20)

P = cardioid(closure);
plot(P(1,:), P(2,:), 'LineStyle','-.','LineWidth', 4);

grid minor;

%% simulation
tfin = 2;
dt=1e-3;
x0 = zeros([3, 1]);
%x0=ones([3,1])*pi/2;
%x0 =IK(xCurled, yCurled)

disp('Simulating...')
tic
[t,x] = ode45(@sim, 0:dt:tfin, x0);
disp(['Done!', num2str(toc)])

%% drawing
lines = [];
for k=1:length(t)
    %if ~eq(mod(k, 10), 0)
    %    continue
    %end

    delete(lines)
    lines = draw_robot(x(k,:));
    pause(0.01)
end

%% Functions
function P =cardioid(cl)
    global L1 L2 L3
    
    tfin = atan2(L1+2*L2+L3, L3-L1);
    t = tfin*(ones(size(cl))-cl);
    a = 0.5*(L3-L1)/ ( (1-cos(tfin)) *cos(tfin) );
    x = L1-L3 + 2*a*(1-cos(t)).*cos(t);
    y = -L2 + 2*a*(1-cos(t)).*sin(t);
    P = [x; y];
end

function lines=draw_robot(q)
    global L1 L2 L3
    j1x = L1*sin(q(1));
    j1y = L1*cos(q(1));
    j2x = j1x+L2*sin(q(1)+q(2));
    j2y = j1y+L2*cos(q(1)+q(2));
    j3x = j2x+L3*sin(q(1)+q(2)+q(3));
    j3y = j2y+L3*cos(q(1)+q(2)+q(3));

    l1 = line([0, j1x], [0, j1y], 'LineStyle','-', 'LineWidth',1.5,'Color','red');
    l2 = line([j1x, j2x], [j1y, j2y], 'LineStyle','-','LineWidth',1.5, 'Color','blue');
    l3 = line([j2x, j3x], [j2y, j3y], 'LineStyle','-', 'LineWidth',1.5,'Color','green');

    lines = [l1; l2; l3];
end

% Robot
% simulation
function dx=sim(t, x)
    global Jtask q tfin R

    J = double(subs(Jtask, q, x));
    J_ = pinv(J);
    
    % cl = min(1, t/(tfin-0.5));
    cl = 0.5 * (1 - cos(6*pi*t/tfin));
    % compute closure, but movement must end 0.5s before end of simulation

    % null space term to keep joints in [0, pi/2]. minus sign to minimize
    % distance from center of joint limits
    % max at pi/2
    dx0 = -[
        (4*x(1) - pi)/(3*pi^2);
        (4*x(2) - pi)/(3*pi^2);
        (4*x(3) - pi)/(3*pi^2)
        ];

    P = cardioid(cl);
    dx = J_*(50*(P-f(x))) + 120*(eye(3) - J_*J)*dx0;
end

% DK
function f=f(q)
    global L1 L2 L3
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    x = L1*sin(q1)+L2*sin(q1+q2)+L3*sin(q1+q2+q3);
    y = L1*cos(q1)+L2*cos(q1+q2)+L3*cos(q1+q2+q3);
    f = [x;y];
end

%IK
function q=IK(x,y)
global L1 L2 L3

%alpha = atan2(y, x);
%beta = pi/2-alpha;
beta = atan2(x, y);
c2 = ((x-L3*sin(beta))^2 + (y-L3*cos(beta))^2 - L1^2 - L2^2)/(2*L1*L2);

if ismembertol(c2, 1) || ismembertol(c2, -1)
    s2 = 0;
else
    s2 = -sqrt(1-c2^2);
end 

A = [L1+L2*c2, L2*s2; -L2*s2 L1+L2*c2];
b = [x-L3*sin(beta); y-L3*cos(beta)];
s1c1 = A\b;
    
q1 = atan2(s1c1(1), s1c1(2));
q2 = atan2(s2, c2);
q3 = pi/2-atan2(y,x)-q1-q2;
%q1 = pi/2

q = [q1; q2; q3];
%q = mod(q+ones([3,1])*2*pi, 2*pi)
end
