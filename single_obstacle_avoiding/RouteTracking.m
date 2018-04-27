function RouteTracking()
    clear all
    clc
    %% initialization
    N = 8;
    T=0.2;%02;
    xref = zeros(N,4);
    wind = [0, 0, 0];
    V = 18;
    mpciterations = 2500;
    time = [];
    xstate = [];
    ucontrol = [];
    xRef = [];
    X = [];
    Y = [];
    XC = [];
    YC = [];
    t0 = 0;
    x = 30;
    y = 0;
    psi = 0;
    s = 10;
    u0 = 0.5 * zeros(1,N);
    %u0 = [19,19,19;0.5,0.5,0.5];
    x0 = zeros(1,3);
    step = V * T;
    numOfObst = 2;
    obst = zeros(5,numOfObst);
    safeRadOfUAV = 20;

    %% B spline
    ctlp=[-124.3241,-124.3178,-124.3150,-124.3171,-124.3244;48.2669,48.2681,48.2646,48.2610,48.2622];
    xctlp=[0,0,0,0,0];
    yctlp=[0,0,0,0,0];
    for i=1:5 
       [xctlp(i),yctlp(i)]=ConvertLL2XY(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
    end
    splinelength=2508.618390858;
    
    obst(1,1) = xctlp(2);
    obst(2,1) = yctlp(2);
    obst(3,1) = 10; % 障碍物的半径
    obst(4,1) = 0;
    obst(5,1) = 0;
     obst(1,2) = xctlp(3);
    obst(2,2) = yctlp(3);
    obst(3,2) = 10; % 障碍物的半径
    obst(4,2) = 0;
    obst(5,2) = 0;
    
    %% The iterative process
    mpciter = 1;
    while(mpciter < mpciterations)
        if s >= splinelength
            s=0;
        end
        %%%%% The prediction of the N reference points 
        count = 1;
        while count <= N
            stemp = s + (count - 1) * step;
            xref(count,:) = BSpline(stemp);
            count = count + 1;
        end
        % Step (1) of the NMPC algorithm: Obtain new initial value
        x0 = [x, y, psi];
        % Step (2) of the NMPC algorithm: Solve the optimal control problem
        [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
                (@runningcosts, @terminalcosts, @obstCost, @constraints, ...
                @terminalconstraints, @linearconstraints, @system, ...
                N, t0, x0, u0, T, wind, xref, obst, numOfObst, V);
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
        % Step (3) of the NMPC algorithm: Apply control to process
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
        if ds > 50  % 这个前推的条件需要商榷
                s = s;
        else
                s = s + 20;
        end  
        mpciter = mpciter+1;  
    end

    %% Plot 
    figure
    plot(xctlp,yctlp,':'); % 绘制控制多边形；
    %plot(x,y,':'); % 绘制控制多边形；
    hold on; % 默认为hold off，此命令用来保留控制多边形的图形；

    plot(X,Y,'b')
    hold on;
    plot(XC,YC,'r')
    hold on;
    for cir = 1:numOfObst
        rectangle('Position',[obst(1,cir)-safeRadOfUAV,obst(2,cir)-safeRadOfUAV,2*safeRadOfUAV,2*safeRadOfUAV],'Curvature',[1,1],'linewidth',1);
        hold on;
    end
    axis equal;
end

%% processing cost function
function cost = runningcosts(x, xd, u)
    cost = norm(x - xd, 2)^2;%+norm(u,2)^2;
end

%% terminal cost function
function cost = terminalcosts(t, x)

    cost = 0.0;
end

%% obstacle collision cost function
function cost = obstCost(x, obst_pre, numOfObst)
    k1 = 50;
    k2 = 25;
    sigma = 200;
    cost = 0.0;
    for j = 1: numOfObst
        d_2 = (obst_pre(1,j) -  x(1))^2 + (obst_pre(2,j) -  x(2))^2;
        if d_2 > 40000
            d_2 = 0;
        end
        J_obs = k1 * exp(-d_2/(sigma^2));
        cost = cost + J_obs;
    end
end
%% constraints
% 需要传入的障碍物信息为第k步的所有障碍物的状态信息
% 计算得出的约束函数应该是所有障碍物的状态在同一时刻引起的约束
function [c,ceq] = constraints(t, x0, x, u, obst, numOfObst, T, wind, V)
    d_safe = 30;
    c = 0;
    for j = 1 : numOfObst
        d_2 = (obst(1,j) -  x(1))^2 + (obst(2,j) -  x(2))^2;
        c_add = d_safe^2 - d_2;
        if d_2 > 40000
            c_add = 0;
        end
        c   = c + c_add;
    end
    ceq = [];
    %ceq(1,1) = x(1) - x0(1) - T/2 * V *(cos(x0(3)) + cos(x0(3) + u)) + T * wind(1);
    %ceq(2,1) = x(2) - x0(2) - T/2 * V *(sin(x0(3)) + sin(x0(3) + u)) + T * wind(2);
    %ceq(3,1) = x(3) - x0(3) - T * u;
    
    
end

%% terminal constraints
function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
end

%% linear constraints
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

%% system model
function x_next = system(t, x, u, T, w, V)
   % v = 19;
    x_next(1) = x(1) + T/2 * (V * (cos(x(3)) + cos(x(3) + u)) + 2 * w(1));
    x_next(2) = x(2) + T/2 * (V * (sin(x(3)) + sin(x(3) + u)) + 2 * w(2));
    x_next(3) = x(3) + T/2 * 3 * u + 2 * w(3);
    %x_next(1) = x(1) + T/2 * (u(1) * (cos(x(3)) + cos(x(3) + u(2))) + 2 * w(1));
    %x_next(2) = x(2) + T/2 * (u(1) * (sin(x(3)) + sin(x(3) + u(2))) + 2 * w(2));
    %x_next(3) = x(3) + T/2 * 3 * u(2) + 2 * w(3);
end

%% refresh the new predictive control input to u0
function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end