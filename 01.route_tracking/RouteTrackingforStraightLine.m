function RouteTrackingforStraightLine()
clear all
clc
%% global veriables
N = 5;
T=0.1;
xref = zeros(N,4);
wind = [0, 0, 0];
V = 19;
mpciterations = 2200;
time = [];
xstate = [];
ucontrol = [];
xRef = [];
X = [];
Y = [];
XC = [];
YC = [];
%% initial states and parameters
t0 = 0;
x = 30;
y = 0;
psi = 0;
s = 10;
u0 = 0.1 * ones(1,N);
%u0 = [19,19,19;0.5,0.5,0.5];
x0 = zeros(1,3);
step = V * T;

%% B spline
%ctlp=[-124.3241,-124.3178,-124.3150,-124.3171,-124.3244;48.2669,48.2681,48.2646,48.2610,48.2622];
%xctlp=[0,0,0,0,0];
%yctlp=[0,0,0,0,0];
%for i=1:5 
%   [xctlp(i),yctlp(i)]=ConvertLL2XY(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
%end
splinelength=2508.618390858;

 

%% The iterative process
mpciter = 1;
while(mpciter < mpciterations)
    if s >= splinelength
        s=0;
        break
    end
    %%%%% The prediction of the N reference points 
    count = 1;
    while count <= N
        stemp = s + (count - 1) * step;
        [xref(count,1),xref(count,2),xref(count,3)] = straightLine(stemp)
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
            (@runningcosts, @terminalcosts, @constraints, ...
            @terminalconstraints, @linearconstraints, @system, ...
            N, t0, x0, u0, T, wind, xref);
    %   Store closed loop data
    time = [ time; t0 ];
    xstate = [ xstate; x0 ];
    ucontrol = [ ucontrol u_new(:,1) ];
    
    %Ds(mpciter)=ds;
    %Time(mpciter)=T*(mpciter-1);
    X(mpciter)=x;
    Y(mpciter)=y;
    %S(mpciter)=s;
    XC(mpciter)=xref(1,1);
    YC(mpciter)=xref(1,2);
    %KSI(mpciter) = psi;        
      
    %   Prepare restart
    u0 = shiftHorizon(u_new);
    t0 = t0 + T;
    %% Step (3) of the NMPC algorithm: Apply control to process
    %x = x0(1) + T/2 * (u_new(2,1) * (cos(x0(3)) + cos(x0(3) + u_new(1,1))) + 2 * wind(1));
    %y = x0(2) + T/2 * (u_new(2,1) * (sin(x0(3)) + sin(x0(3) + u_new(1,1))) + 2 * wind(2));
    %psi = x0(3) + T/2 * 3 * u_new(1,1) + 2 * wind(3);
    x = x0(1) + T/2 * (V * (cos(x0(3)) + cos(x0(3) + u_new(1))) + 2 * wind(1));
    y = x0(2) + T/2 * (V * (sin(x0(3)) + sin(x0(3) + u_new(1))) + 2 * wind(2));
    psi = x0(3) + T/2 * 3 * u_new(1) + 2 * wind(3);
    if psi>pi
            psi = psi-2*pi;
    elseif psi<-pi
            psi = psi+2*pi;
    end	
    ds = sqrt((x-xref(1,1))^2 + (y-xref(1,2))^2);
    if ds > 50
            s = s;
    else
            s = s + 30;
    end  
    mpciter = mpciter+1;  
end
save('D:\03.code\data_analysis\RT_straight_line\SL_N5T01.mat','X','Y');
%% Plot 
figure
%plot(xctlp,yctlp,':'); % 绘制控制多边形；
%plot(x,y,':'); % 绘制控制多边形；
%hold on; % 默认为hold off，此命令用来保留控制多边形的图形；

plot(X,Y,'b')
hold on;
plot(XC,YC,'r')
hold on;
axis equal;

end

% need thinking % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = runningcosts(x, xd, u)
    cost = norm(x - xd, 2)^2;%+norm(u,2)^2;
end

function cost = terminalcosts(t, x)

    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u)
    c   = [];
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    %lb  = [15;-3];
    %ub  = [23;3];
    lb  = [-5];
    ub  = [5];
end

function [x, y, psi] = straightLine(s)
    k = 3;
    x = s / sqrt(1+k^2);
    y = k * x;
    psi = atan(k);
end

function x_next = system(t, x, u, T, w)
    v = 19;
    x_next(1) = x(1) + T/2 * (v * (cos(x(3)) + cos(x(3) + u)) + 2 * w(1));
    x_next(2) = x(2) + T/2 * (v * (sin(x(3)) + sin(x(3) + u)) + 2 * w(2));
    x_next(3) = x(3) + T/2 * 3 * u + 2 * w(3);
    %x_next(1) = x(1) + T/2 * (u(1) * (cos(x(3)) + cos(x(3) + u(2))) + 2 * w(1));
    %x_next(2) = x(2) + T/2 * (u(1) * (sin(x(3)) + sin(x(3) + u(2))) + 2 * w(2));
    %x_next(3) = x(3) + T/2 * 3 * u(2) + 2 * w(3);
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end