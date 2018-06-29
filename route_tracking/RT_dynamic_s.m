function RT_dynamic_s()
clear all
clc
%% global veriables
N = 5;
T = 0.05;
xref = zeros(N,4);
wind = [0, 0, 0];
V = 19;
Vd = 19;
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
x0 = zeros(1,3);


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
    
    step = Vd * T;
    
    if s >= splinelength
        s=0;
        break
    end
    %%%%% The prediction of the N reference points 
    count = 1;
    while count <= N
        stemp = s + (count - 1) * step;
        xref(count,:) = BSpline(stemp);
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
    SSS(mpciter)=xref(1,4);
      
    %   Prepare restart
    u0 = shiftHorizon(u_new);
    t0 = t0 + T;
    
    %% Step (3) of the NMPC algorithm: Apply control to process
    x = x0(1) + T/2 * (V * (cos(x0(3)) + cos(x0(3) + u_new(1))) + 2 * wind(1));
    y = x0(2) + T/2 * (V * (sin(x0(3)) + sin(x0(3) + u_new(1))) + 2 * wind(2));
    psi = x0(3) + T/2 * 3 * u_new(1) + 2 * wind(3);
    if psi>pi
            psi = psi-2*pi;
    elseif psi<-pi
            psi = psi+2*pi;
    end	
    
    %% Vd 
    
    
    psi_d = xref(1,3);
    W_ex = wind(1)*cos(psi_d) + wind(2)*sin(psi_d);
    W_ey = -wind(1)*sin(psi_d) + wind(2)*cos(psi_d);
    l_curve = xref(1,4);
    x_d = xref(1,1);
    y_d = xref(1,2);
    error = [cos(psi_d)  -sin(psi_d);sin(psi_d) cos(psi_d)]'*([x;y]-[x_d;y_d]);
    es = error(1);
    ed = error(2);
    e_psi = psi - psi_d;
    
    temp = (es*( V*cos(e_psi) + W_ex) + ed*( V*sin(e_psi) + W_ey)) / (es + e_psi*l_curve);
    if temp < 0
        V0 =abs((es*( V*cos(e_psi) + W_ex) + ed*( V*sin(e_psi) + W_ey)) / (es + e_psi*l_curve))+0.1;
    else
        V0 = 0.1;    
    end
    Vd = (es*( V*cos(e_psi) + W_ex) + ed*( V*sin(e_psi) + W_ey)) / (es + e_psi*l_curve) + V0
    %Vd = 9;
    
    ds = sqrt((x-xref(1,1))^2 + (y-xref(1,2))^2);
    if ds > 15%30
            s = s;
    else
            %s = s + 20;
            s = s + Vd * 3 * T;
    end 
    mpciter = mpciter+1;  
end

save('D:\03.code\data_analysis\RT_s_forward\N5T005_test.mat','X', 'Y');

%figure
%plot(SSS);
%% Plot 
figure(2)
plot(xctlp,yctlp,':'); % 绘制控制多边形；
%plot(x,y,':'); % 绘制控制多边形；
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；

plot(X,Y,'b')
hold on;
plot(XC,YC,'r')
hold on;
grid on;
axis equal;

end

% need thinking % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = runningcosts(x, xd, u)
    cost = norm(x - xd, 2)^2;
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
    lb  = [-5];
    ub  = [5];
end


function x_next = system(t, x, u, T, w)
    v = 19;
    x_next(1) = x(1) + T/2 * (v * (cos(x(3)) + cos(x(3) + u)) + 2 * w(1));
    x_next(2) = x(2) + T/2 * (v * (sin(x(3)) + sin(x(3) + u)) + 2 * w(2));
    x_next(3) = x(3) + T/2 * 3 * u + 2 * w(3);
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end