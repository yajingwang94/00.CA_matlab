与function multi_RouteTracking()
clear all
clc
%% global veriables
N = 5;
T = 0.05;
xref = zeros(N,4);

xref_1 = zeros(N,4);

wind = [0, 0, 0];
%wind = [2.12132034,2.12132034,0];
%wind = [3.53533906, 3.535533906,0];
%wind = [5.65685425, 5.65685425,0];
%wind = [7.07106781, 7.07106781,0];
%wind = [1, 1, 0];
V = 19;
mpciterations = 500;
xRef = [];
X = [];
Y = [];
XC = [];
YC = [];

X1 = [];
Y1 = [];
XC1 = [];
YC1 = [];
%% initial states and parameters
t0 = 0;
x = 25;
y = 0;
psi = 0;
s = 10;
u0 = 0.3 * ones(1,N);
x0 = zeros(1,3);

x_1 = -100;
y_1 = 0;
psi_1 = 0;
s_1 = 10;
u0_1 = 0.3 * ones(1,N);
x0_1 = zeros(1,3);

step = V * T;

num_of_others = 1;
other_state = [];
other_u0 = [];

%% B spline
ctlp=[-124.3241,-124.3178,-124.3150,-124.3171,-124.3244;48.2669,48.2681,48.2646,48.2610,48.2622];
xctlp=[0,0,0,0,0];
yctlp=[0,0,0,0,0];
for i=1:5 
   [xctlp(i),yctlp(i)]=ConvertLL2XY(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
end
splinelength=2508.618390858;


%% The iterative process
mpciter = 1;
while(mpciter < mpciterations)
    if s >= splinelength
        s = 0;
		break;
    end
	if s_1 >= splinelength
        s_1 = 0;
		break;
    end
    %%%%% The prediction of the N reference points 
    count = 1;
    while count <= N
        s_temp = s + (count - 1) * step;
        xref(count,:) = BSpline(s_temp);
		
		s_temp1 = s_1 + (count - 1) * step;
        xref_1(count,:) = BSpline(s_temp1);
		
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];
	x0_1 = [x_1, y_1, psi_1];
	
	
	other_state(1,:) = x0_1;  %%每一行代表一架邻机
	other_u0(1,:) = u0_1;
	
	
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, ...
            N, t0, x0, u0, T, wind, xref, other_state, other_u0);
    %   Store closed loop data
	
	other_state(1,:) = x0;  %%每一行代表一架邻机
	other_u0(1,:) = u0;
	
	[u_new_1, V_current_1, exitflag_1, output_1] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @coopCost, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, ...
            N, t0, x0_1, u0_1, T, wind, xref_1, other_state, other_u0);
	
    %Time(mpciter)=T*(mpciter-1);
    X(mpciter) = x;
    Y(mpciter) = y;
    XC(mpciter) = xref(1,1);
    YC(mpciter) = xref(1,2);
	
	X1(mpciter) = x_1;
    Y1(mpciter) = y_1;
    XC1(mpciter) = xref_1(1,1);
    YC1(mpciter) = xref_1(1,2);
    %KSI(mpciter) = psi;        
      
    %   Prepare restart
    u0 = shiftHorizon(u_new);
	u0_1 = shiftHorizon(u_new_1);
    t0 = t0 + T;
    %% Step (3) of the NMPC algorithm: Apply control to process
    x = x0(1) + T/2 * (V * (cos(x0(3)) + cos(x0(3) + u_new(1))) + 2 * wind(1));
    y = x0(2) + T/2 * (V * (sin(x0(3)) + sin(x0(3) + u_new(1))) + 2 * wind(2));
    psi = x0(3) + T/2 * 3 * u_new(1) + 2 * wind(3);
	
	x_1 = x0_1(1) + T/2 * (V * (cos(x0_1(3)) + cos(x0_1(3) + u_new_1(1))) + 2 * wind(1));
    y_1 = x0_1(2) + T/2 * (V * (sin(x0_1(3)) + sin(x0_1(3) + u_new_1(1))) + 2 * wind(2));
    psi_1 = x0_1(3) + T/2 * 3 * u_new_1(1) + 2 * wind(3);
	
    if psi > pi
            psi = psi-2*pi;
    elseif psi < -pi
            psi = psi+2*pi;
    end	
	
	if psi_1 > pi
            psi_1 = psi_1-2*pi;
    elseif psi_1 < -pi
            psi_1 = psi_1+2*pi;
    end
	
    ds = sqrt((x - xref(1,1))^2 + (y - xref(1,2))^2);
    if ds > 30
            s = s;
    else
            s = s + 20;
    end  
	
	ds_1 = sqrt((x_1 - xref_1(1,1))^2 + (y_1 - xref_1(1,2))^2);
    if ds_1 > 30
            s_1 = s_1;
    else
            s_1 = s_1 + 20;
    end
    mpciter = mpciter+1;  
end

save('D:\code\multi_uav_test\2_N2T005.mat','X','Y','X1','Y1');
%% Plot 
figure
plot(xctlp,yctlp,':'); % 绘制控制多边形；
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；

plot(X,Y,'b')
hold on;
plot(X1,Y1,'g')
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

function cost = coopCost(x, x_others)
    k1 = 50;
    %k2 = 25;
    sigma = 200;
    cost = 0.0;
	numOfOthers = size(x_others,1);
    for j = 1: numOfOthers
        d_2 = (x_others(j,1) -  x(1))^2 + (x_others(j,2) -  x(2))^2;
        if d_2 > 40000
            d_2 = 0;
        end
        J_coop = k1 * exp(-d_2/(sigma^2));
        cost = cost + J_coop;
    end
end

function [c,ceq] = collAvoid_constraints(system, x, t0, N, T, other_state, other_u0, wind)
	x_other = [];
    c = [];
	x_other(1,:) = other_state;
    for k = 1:N
        x_other(k+1,:) = system(t0, x_other(k,:), other_u0(k), T, wind);
    end
	
	d_safe = 40;
	for i = 1:N
		dis_2 = (x(i+1,1) - x_other(i+1,1))^2 + (x(i+1,2) - x_other(i+1,2))^2;
		
		c_add = d_safe^2 - dis_2;
        c   = [c; c_add];
	end
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
    lb  = [-0.3];
    ub  = [0.3];
end


function x_next = system(t, x, u, T, wind)
    v = 19;
    x_next(1) = x(1) + T/2 * (v * (cos(x(3)) + cos(x(3) + u)) + 2 * wind(1));
    x_next(2) = x(2) + T/2 * (v * (sin(x(3)) + sin(x(3) + u)) + 2 * wind(2));
    x_next(3) = x(3) + T/2 * 3 * u + 2 * wind(3);
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end