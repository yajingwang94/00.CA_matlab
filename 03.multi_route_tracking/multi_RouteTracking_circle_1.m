function multi_RouteTracking_circle()
clear all
clc
%% global variebles
N = 5;
T = 0.05;
xref = zeros(N,4);
wind = [0, 0, 0];
%wind = [2.12132034,2.12132034,0];
%wind = [3.53533906, 3.535533906,0];
%wind = [5.65685425, 5.65685425,0];
%wind = [7.07106781, 7.07106781,0];
%wind = [1, 1, 0];
V = 19;
mpciterations =1000;
xRef = [];
X1 = [];
Y1 = [];
XC1 = [];
YC1 = [];
PSI1 = [];
PSIF1 = [];
JVALUE1 = [];
UCON1= [];
FLAG1 = [];
Dis1 = [];
COOPCost1 = [];
X12 = [];
Y12 = [];
%% initial states and parameters
t0 = 0;
x = 300;
y = 0;     %初始时刻的位置在北东地坐标系下的表示
psi = 0;
u0 = 0.3 * ones(1,N);
x0 = zeros(1,3);
step = V * T;
num_of_others = 1;
other_state = [];
other_u0 = [];
Dis = 0;
COOPCost = 0;

%% 圆轨迹方程
     R = 300;
     splinelength = 2 * pi * R;     
     s = 10;    
    
     %theta = s / R;
     % if theta > pi
     %           theta = theta - 2 * pi;
     %   elseif theta < - pi
     %           theta = theta + 2 * pi;
     %   end	  
     %xf = R * cos(theta);
     %yf = R * sin(theta);
     %xprim = -sin(theta);
     %yprim = cos(theta);
     %psif = atan2(yprim,xprim);
     %if theta <= 0 
     %   psif = theta + pi/2;
    %elseif theta <= pi/2
     %   psif = theta + pi/2;
    %elseif theta > pi/2
     %       psif = 3*pi/2 - theta;
     %   end
     
     
     %psif = -pi - theta;
     
%% communication     
    receive_server = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
%Open a connection. This will not return until a connection is received.
%set(t,'BytesAvailable',100)
    fopen(receive_server);
    %send_client = tcpip('localhost', 30001, 'NetworkRole', 'client');
    %fopen(send_client)
    send_client = tcpip('localhost', 30001, 'NetworkRole', 'client');
    fopen(send_client)

     


%% The iterative process
mpciter = 1;
while(mpciter < mpciterations)
    if s >= splinelength
        s = 10;  %逆时针方向，s增大 
		%break;
    end
	
    %%%%% The prediction of the N reference points 
    count = 1;
    while count <= N
        s_temp = s + (count - 1) * step;
        [xref(count,1), xref(count,2), xref(count,3)] = getRef(s_temp, N , R);		
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];    %每一时刻初始状态在北东地坐标系下的坐标表示
	%x0_1 = [x_1, y_1, psi_1];
	
	 %[other_state, other_u0] = server1();
    %rec = fread(receive_server, 8);
     rec = fscanf(receive_server, '%f', 8)
     size = whos('rec')
     if(size.size>0)
     other_state(1,:) = rec(1:3);
     other_u0(1,:) = rec(4:8);
     X12(mpciter)=rec(1);
    Y12(mpciter)=rec(2);
     end   
     
	%other_state(1,:) = x0_1;  %%每一行代表一架邻机
	%other_u0(1,:) = u0_1;
    %num = zeros(1,2);
    
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output, Dis, COOPCost] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @coopCost, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, @coopCostTest,...
            N, t0, x0, u0, T, wind, xref, other_state, other_u0);
    %   Store closed loop data
	u0 = shiftHorizon(u_new);
    %Time(mpciter)=T*(mpciter-1);
    X1(mpciter) = x;
    Y1(mpciter) = -y; % Y records the coordination in X-Y coordination system
    PSI1(mpciter) = psi;
    XC1(mpciter) = xref(1,1);
    YC1(mpciter) = xref(1,2);
    PSIF1(mpciter) = xref(1,3);
    JVALUE1(mpciter) = V_current;
    UCON1(mpciter) = u_new(1);
    FLAG1(mpciter) = exitflag;
    Dis1(mpciter) = Dis;
    COOPCost1(mpciter) = COOPCost;
    
  
    send_data = zeros(1,6);
    send_data(1:3)=[x,y,psi];
    send_data(4:8)=u0;
    send_data
    %fwrite(send_client, send_data)
    %fwrite(send_client, send_data) 
    fprintf(send_client, '%f',send_data)
    %   Prepare restart
    
	
    t0 = t0 + T;
    %% Step (3) of the NMPC algorithm: Apply control to process
    x = x0(1) + T/2 * (V * (cos(x0(3)) + cos(x0(3) + u_new(1))) + 2 * wind(1));
    y = x0(2) + T/2 * (V * (sin(x0(3)) + sin(x0(3) + u_new(1))) + 2 * wind(2));
    psi = x0(3) + T/2 * 3 * u_new(1) + 2 * wind(3);
		
    if psi > pi
            psi = psi-2*pi;
    elseif psi < -pi
            psi = psi+2*pi;
    end	
		
    ds = sqrt((x - xref(1,1))^2 + (y - xref(1,2))^2);
    if ds > 40
            s = s;
    else
            s = s + 20;
    end  
	    mpciter = mpciter+1;  
end

fclose(receive_server);
fclose(send_client);

save('D:\03.code\data_analysis\multi_circle\UAV1_test0830_1.mat','X1','Y1', 'PSI1', 'XC1','YC1','PSIF1', 'JVALUE1','UCON1', 'FLAG1', 'Dis1', 'COOPCost1', 'X12', 'Y12' );
%% Plot 
figure
%plot(xctlp,yctlp,':'); % 绘制控制多边形；
%hold on; % 默认为hold off，此命令用来保留控制多边形的图形；

plot(X1,Y1,'b')
hold on;
%plot(X1,Y1,'g')
%hold on;
plot(XC1,YC1,'r')
hold on;
axis equal;

end

% need thinking % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = runningcosts(x, xd, u)
    cost = (x(1) - xd(1))^2 + (x(2) - xd(2))^2;%+norm(u,2)^2;
end

function cost = terminalcosts(t, x)

    cost = 0.0;
end

function cost = coopCost(x, x_others)
    k1 = 100;
    %k2 = 25;
    sigma = 100;%200;
    cost = 0.0;
	numOfOthers = size(x_others,1);
    for j = 1: numOfOthers
        d_2 = (x_others(j,1) -  x(1))^2 + (x_others(j,2) -  x(2))^2;
        if d_2 > 40000
            d_2 = 0;
            J_coop = 0;
        else
            J_coop = k1 * exp(-d_2/(sigma^2));
        end
        cost = cost + J_coop;
    end
end

function [dis, COOPCost] = coopCostTest(x0, other_state)
    k1 = 100;
    %k2 = 25;
    sigma = 100;%200;
	%numOfOthers = size(x_others,1);
    COOPCost = zeros(1,3); %当前状态的代价
    %dis = zeros(1,3);
    %for j = 1: numOfOthers
        d_2 = (other_state(1,1) -  x0(1))^2 + (other_state(1,2) -  x0(2))^2;
        dis = sqrt(d_2);
        if d_2 > 40000
            d_2 = 0;
            J_coop = 0;
        else
            J_coop = k1 * exp(-d_2/(sigma^2));
        end;
        COOPCost = J_coop;
       % cost = cost + J_coop;
    %end
end

function [c,ceq] = collAvoid_constraints(system, x,t0, N, T, x_other)
    c = 0;
	d_safe = 40;
    num_of_others = size(x_other, 1);
	for i = 1:num_of_others
		dis_2 = (x(1) - x_other(i,1))^2 + (x(2) - x_other(i,2))^2;
		
		c_add = d_safe^2 - dis_2;
        c   = c + c_add;
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

function [xref, yref, psiref] = getRef(s, N, R)
% 逆时针方向计算参考点的坐标和参考航向角reverse direction of the circle
     theta = s / R;
      if theta > pi
                theta = theta - 2 * pi;
        elseif theta < - pi
                theta = theta + 2 * pi;
        end	
     xref = R * cos(theta);
     yref = - R * sin(theta); % use the N-E-E coordination
     %xprim = -sin(theta);
     %yprim = cos(theta);
     %psiref = atan2(yprim,xprim);
     if theta <= 0 
        psiref = - pi/2 - theta ;
    elseif theta <= pi/2
        psiref = - pi/2 - theta;
     else
            psiref = 3*pi/2 - theta;
        end
end

function [xref, yref, psiref] = getRef_2(s, N, R)
% along the direction of circle
     theta = s / R;
      if theta > pi
                theta = theta - 2 * pi;
        elseif theta < - pi
                theta = theta + 2 * pi;
        end	
     xref = R * cos(theta);
     yref = R * sin(theta);
     xprim = -sin(theta);
     yprim = cos(theta);
    % psiref = atan2(yprim,xprim);
     psiref = atan2(yprim,xprim) + 2*pi;
     if psiref > pi
                psiref = psiref - 2 * pi;
     elseif psiref < - pi
                psiref = psiref + 2 * pi;
     end
     
end