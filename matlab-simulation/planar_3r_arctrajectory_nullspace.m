clc
clear

%% preliminary computations and plots
global L1 L2 L3 L
global xExtended yExtended xCurled yCurled
global angle angleE angleC xM yM R

% index
% lengths
% L1 = 0.0435;
% L2 = 0.0333;
% L3 = 0.0204;
% R=0.068

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
L1 = 0.0337
L2 = 0.0252
L3 = 0.0135

R = 0.052

L=L1+L2+L3;


closure = 0:0.01:1;
xExtended=0
yExtended=L1+L2+L3

xCurled=L1-L3
yCurled=-L2

xM = 0.5*(xCurled+xExtended)
yM = 0.5*(yCurled+yExtended)

% R = 0.5*sqrt((xExtended-xCurled)^2 + (yExtended-yCurled)^2);
% 
% angleE = atan2(yExtended-yM, xExtended-xM)
% angleC = atan2(yCurled-yM, xCurled-xM)
% angle = angleE - angleC;

% xTrj = -R*cos(angleE + angle*closure) + xM;
% yTrj = -R*sin(angleE + angle*closure) + yM;
% xTrj = R*sin(pi*closure);
% yTrj = R*cos(pi*closure) +yM;

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

% arc to be traced by the finger
%plot(xTrj(:), yTrj(:), 'LineStyle','-.','LineWidth', 4);
%plot(xTrj(:), yTrj(:), 'Marker','+', 'MarkerSize', 40);

P = two_point_arc(R, closure)
plot(P(1,:), P(2,:), 'LineStyle','-.','LineWidth', 4);

%line([xM, xM+R*cos(angleE)], [yM, yM+R*sin(angleE)])
%line([xM, xM+R*cos(angleC)], [yM, yM+R*sin(angleC)])

% for k=1:length(c)
%     [d, alpha] = closure_to_distance(r,c(k));
%     line([0,d*cos(alpha)], [0,d*sin(alpha)]);
% end
%legend()

grid minor;

%% simulation
tfin = 3;
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

% lines = [];
% for k=1:length(closure)
%     %if ~eq(mod(k, 10), 0)
%     %    continue
%     %end
% 
%     delete(lines)
%     lines = draw_robot(ones(3,1)*pi/2*closure(k));
%     pause(0.01)
% end

%draw_robot(ones(3,1)*pi/2)

% figure()
% pp = [];
% for k=1:length(x)
%     pp = [pp, double(subs(p, q', x(k, :)))];
% end
% hold on
% [d, ~] = closure_to_distance(0.08, t/tfin);
% plot(t, d)
% plot(t, pp)
% figure()
% hold on
% plot(t, x(:,1))
% plot(t, x(:,2))
% plot(t, x(:,3))

%% Functions
function [P, gamma, gamma_start, gamma_end] =two_point_arc(R, cl)
    global xExtended yExtended xCurled yCurled

    xp1 = xExtended;
    yp1 = yExtended;
    xp2 = xCurled;
    yp2 = yCurled;
    
    D = yp2^2-yp1^2+xp2^2-xp1^2;
    E = D/(2*(xp2-xp1));
    C = (yp1-yp2)/(xp2-xp1);
    
    a = C^2 + 1;
    b = 2*(E*C - E*C*xp1 - yp1);
    c = E^2 - R^2 + xp1^2 + yp1^2 -2*xp1*E;
    
    yO = (-b - sqrt(b^2-4*a*c))/(2*a);
    xO = E+yO*C;
   
    gamma_start = atan2(xExtended-xO, yExtended-yO);
    gamma_end = atan2(xCurled-xO, yCurled-yO);
    gamma = gamma_end-gamma_start;
    xArc = R*sin(gamma_start +gamma*cl) + xO;
    yArc = R*cos(gamma_start +gamma*cl) + yO;
    %gamma = gamma_start-gamma_end;
    %xArc = R*sin(gamma_end+gamma*cl) + xO;
    %yArc = R*cos(gamma_end +gamma*cl) + yO;
    P=[xArc;yArc];
end

function [P, dP, gamma, gamma_start, gamma_end] =two_point_arc_with_tangent(R, cl)
    global xExtended yExtended xCurled yCurled

    xp1 = xExtended;
    yp1 = yExtended;
    xp2 = xCurled;
    yp2 = yCurled;
    
    D = yp2^2-yp1^2+xp2^2-xp1^2;
    E = D/(2*(xp2-xp1));
    C = (yp1-yp2)/(xp2-xp1);
    
    a = C^2 + 1;
    b = 2*(E*C - E*C*xp1 - yp1);
    c = E^2 - R^2 + xp1^2 + yp1^2 -2*xp1*E;
    
    yO = (-b - sqrt(b^2-4*a*c))/(2*a);
    xO = E+yO*C;
   
    gamma_start = atan2(xExtended-xO, yExtended-yO);
    gamma_end = atan2(xCurled-xO, yCurled-yO);
    gamma = gamma_end-gamma_start;
    xArc = R*sin(gamma_start +gamma*cl) + xO;
    yArc = R*cos(gamma_start +gamma*cl) + yO;

    dxArc = R*cos(gamma_start + gamma*cl)*cl;
    dyArc = -R*sin(gamma_start + gamma*cl)*cl;
    %gamma = gamma_start-gamma_end;
    %xArc = R*sin(gamma_end+gamma*cl) + xO;
    %yArc = R*cos(gamma_end +gamma*cl) + yO;
    P=[xArc;yArc];
    dP = [dxArc; dyArc];
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
    % max at 80°
    % dx0 = -[
    %     (27*x(1) - 6*pi)/(16*pi^2);
    %     (27*x(2) - 6*pi)/(16*pi^2);
    %     (27*x(3) - 6*pi)/(16*pi^2)
    %     ];
    %dx = dx0 + J_*(1500*(two_point_arc(0.08, t/tfin)-f(x)) - 15000*J*dx0);
    
    % smaller coeffients -> lower sim time
    %dx = 200*dx0 + J_*(10*(two_point_arc(0.08, t/tfin)-f(x)) - 200*J*dx0);

    % alternative to computation of null space
    [P, dP, ~, ~, ~] = two_point_arc_with_tangent(R, cl);
    %dx = J_*(50*(P-f(x))) + 50*(eye(3) - J_*J)*dx0;

    % crazy thought: keeping the joints within limits is the first task,
    % following the trajectory is in the null space
    dx = diag([2, 20, 2])*dx0 + 50*J_*(P-f(x)); % slightly less crap
    %dx = 5*dx0 + 500*(eye(3) - J_*J)*J_*(dP + 50*(P-f(x))); % this is crap
        
    %dx = J_*1500*(two_point_arc(0.08, t/tfin)-f(x)) ;
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
